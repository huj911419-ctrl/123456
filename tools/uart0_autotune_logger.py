#!/usr/bin/env python3
"""Receive offline auto-tune logs dumped by AutoTuneLog.c.

Run this before the car starts. The car records to RAM while motor_enable is on,
then dumps packets after the run stops:
    AA 55 A0 summary
    AA 55 A1 records
    AA 55 A2 end
It also receives low-rate A3 live breadcrumbs during the run.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import struct
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    print("Missing dependency: pyserial")
    print("Install with: python -m pip install pyserial")
    raise SystemExit(1) from exc


HEADER = b"\xAA\x55"
TYPE_SUMMARY = 0xA0
TYPE_RECORD = 0xA1
TYPE_END = 0xA2
TYPE_LIVE = 0xA3
LEGACY_RECORD_SIZE = 58
RECORD_SIZE_V2 = 67
RECORD_SIZE_V3 = 72
RECORD_SIZE_V4 = 74
RECORD_SIZE_V5 = 83
RECORD_SIZE = 101
RECORD_SIZE_V6 = 117
LIVE_SIZE_V2 = 28
LIVE_SIZE_V3 = 29
LIVE_SIZE = 31
MAX_PAYLOAD_LEN = 2048
TURN_MARK_HARD_ENTER = 0x01
TURN_MARK_YAW_START = 0x02
TURN_MARK_DUTY_START = 0x04
TURN_MARK_RECOVER_BLOCK = 0x08

EXIT_REASON_NAMES = {
    0: "none",
    1: "line",
    2: "yaw",
    3: "coast",
    4: "timeout",
    5: "emergency",
    6: "no_imu",
    7: "ra_timeout",
    8: "recover",
}


@dataclass
class Summary:
    version: int
    record_size: int
    count: int
    overflow: int
    pid_period_ms: int
    sample_period_ms: int
    motor_speed: int
    pid_kp: int
    pid_kd: int
    ra_turn_row: int
    ra_hard_outer: int
    ra_hard_yaw: int


@dataclass
class Record:
    seq: int
    t_ms: int
    pid_tick: int
    base_speed: int
    speed_plan: int
    speed_out: int
    steer_out: int
    err: int
    lookahead: int
    trend: int
    yaw10: int
    ra_yaw10: int
    ra_hard_target10: int
    ra_outer_cmd: int
    yaw_rate_dps: int
    duty_left: int
    duty_right: int
    left_speed: int
    right_speed: int
    ra_timer: int
    ra_hard_cnt: int
    tf_us: int
    inter_us: int
    battery_x10: int
    valid_rows: int
    ra_flag: int
    ra_pre: int
    ra_phase: int
    ra_dir: int
    ra_ip_row: int
    ra_slow_row: int
    ra_turn_row: int
    ra_exit_reason: int
    route_step: int
    route_flag: int
    route_action: int
    post_edge_side: int
    speed_reason: int
    line_lost: int
    inter_type: int
    sym_component: int
    ip_max_row: int
    ip_visual_row: int
    ip_ctrl_row: int
    ip_ctrl_dir: int
    ip_ctrl_score: int
    ip_ctrl_reason: int
    fast_ra_type: int
    fast_front: int
    fast_left: int
    fast_right: int
    line_takeover_ready: int
    line_takeover_cnt: int
    exit_boost_active: int
    exit_boost_cnt: int
    hard_outer_dynamic: int
    yaw_pred: int
    yaw_remain: int
    outer_scale: int
    takeover_valid_rows: int
    takeover_error: int
    takeover_lookahead: int
    takeover_trend: int
    box_top: int
    box_bottom: int
    box_left: int
    box_right: int
    pre_detail: int
    turn_mark: int
    hard_enter_ip: int
    hard_enter_tick: int
    pre_seen_frames: int
    # v6 fields (record_size == 117)
    yaw_total_progress: int = 0
    yaw_hard_progress: int = 0
    visual_exit_ready: int = 0
    visual_stable_cnt: int = 0
    yaw_guard_active: int = 0
    over_turn_guard: int = 0
    line_takeover_speed_cap: int = 0
    turn_assist_active: int = 0
    turn_assist_weight: int = 0
    turn_assist_found_col: int = 0
    inner_min_pct: int = 0
    outer_boost_pct: int = 0
    continuous_turn_mode: int = 0
    exit_reason_verbose: int = 0
    extra_raw_101_117: str = ""


@dataclass
class LiveRecord:
    seq: int
    t_ms: int
    speed_plan: int
    err: int
    lookahead: int
    yaw10: int
    ra_yaw10: int
    yaw_rate_dps: int
    battery_x10: int
    duty_left: int
    duty_right: int
    valid_rows: int
    ra_flag: int
    ra_phase: int
    ra_dir: int
    ra_ip_row: int
    route_step: int
    post_edge_side: int
    speed_reason: int
    line_lost: int
    inter_type: int
    ip_max_row: int


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device:10s} {port.description or ''} {port.hwid or ''}")


def read_exact(ser: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            raise TimeoutError
        data.extend(chunk)
    return bytes(data)


def read_packet(ser: serial.Serial) -> tuple[int, bytes]:
    sync = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError
        sync += b
        if len(sync) > 2:
            del sync[0]
        if bytes(sync) == HEADER:
            break

    meta = read_exact(ser, 3)
    packet_type = meta[0]
    payload_len = (meta[1] << 8) | meta[2]
    if payload_len > MAX_PAYLOAD_LEN:
        raise ValueError(f"bad payload length {payload_len}")
    return packet_type, read_exact(ser, payload_len)


def unpack_summary(payload: bytes) -> Summary:
    if len(payload) != 22:
        raise ValueError(f"summary length {len(payload)} != 22")
    values = struct.unpack(">BBHHHHhhhhhh", payload)
    return Summary(*values)


# V6 (117 bytes) = 101-byte body + 16-byte suffix:
#   hh = yaw_total_progress10, yaw_hard_progress10         (4 B)
#   6B = visual_exit_ready, yaw_guard_active, over_turn_guard,
#        line_takeover_speed_cap, turn_assist_active,
#        turn_assist_weight                                    (6 B)
#   h  = turn_assist_found_col                                (2 B)
#   4B = continuous_turn_mode, exit_reason_verbose,
#        inner_min_pct, outer_boost_pct                       (4 B)
RECORD_FMT_V6_BODY = ">HH16h5H31B3h2B3h5BH3B"   # first 101 bytes
RECORD_FMT_V6_SUFFIX = ">hhBBBBBBhBBBB"         # last 16 bytes


def unpack_records(payload: bytes, summary: Summary) -> list[Record]:
    record_size = summary.record_size
    if record_size not in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5,
                           RECORD_SIZE_V4, RECORD_SIZE_V3,
                           RECORD_SIZE_V2, LEGACY_RECORD_SIZE):
        raise ValueError(f"unsupported record size {record_size}")
    if len(payload) % record_size != 0:
        raise ValueError(f"record packet length {len(payload)} is not multiple of {record_size}")
    records: list[Record] = []
    if record_size == RECORD_SIZE_V6:
        fmt = RECORD_FMT_V6_BODY + RECORD_FMT_V6_SUFFIX.lstrip(">")
    elif record_size == RECORD_SIZE:
        fmt = ">HH16h5H31B3h2B3h5BH3B"
    elif record_size == RECORD_SIZE_V5:
        fmt = ">HH16h5H32BH3B"
    elif record_size == RECORD_SIZE_V4:
        fmt = ">HH16h5H23BH3B"
    elif record_size == RECORD_SIZE_V3:
        fmt = ">HH16h4H23BH3B"
    elif record_size == RECORD_SIZE_V2:
        fmt = ">HH16h4H23B"
    else:
        fmt = ">H14h4H20B"
    for off in range(0, len(payload), record_size):
        vals = struct.unpack_from(fmt, payload, off)
        if record_size == LEGACY_RECORD_SIZE:
            seq = vals[0]
            t_ms = seq * summary.sample_period_ms
            pid_tick = t_ms // summary.pid_period_ms if summary.pid_period_ms else seq
            records.append(
                Record(
                    seq=seq,
                    t_ms=t_ms,
                    pid_tick=pid_tick,
                    base_speed=vals[1],
                    speed_plan=vals[2],
                    speed_out=vals[3],
                    steer_out=vals[4],
                    err=vals[5],
                    lookahead=vals[6],
                    trend=vals[7],
                    yaw10=vals[8],
                    ra_yaw10=vals[9],
                    ra_hard_target10=summary.ra_hard_yaw * 10,
                    ra_outer_cmd=0,
                    yaw_rate_dps=vals[10],
                    duty_left=vals[11],
                    duty_right=vals[12],
                    left_speed=vals[13],
                    right_speed=vals[14],
                    ra_timer=vals[15],
                    ra_hard_cnt=vals[16],
                    tf_us=vals[17],
                    inter_us=vals[18],
                    battery_x10=0,
                    valid_rows=vals[19],
                    ra_flag=vals[20],
                    ra_pre=vals[21],
                    ra_phase=vals[22],
                    ra_dir=vals[23],
                    ra_ip_row=vals[24],
                    ra_slow_row=0,
                    ra_turn_row=0,
                    ra_exit_reason=0,
                    route_step=vals[25],
                    route_flag=vals[26],
                    route_action=vals[27],
                    post_edge_side=vals[28],
                    speed_reason=vals[29],
                    line_lost=vals[30],
                    inter_type=vals[31],
                    sym_component=vals[32],
                    ip_max_row=vals[33],
                    ip_visual_row=0,
                    ip_ctrl_row=0,
                    ip_ctrl_dir=0,
                    ip_ctrl_score=0,
                    ip_ctrl_reason=0,
                    fast_ra_type=0,
                    fast_front=0,
                    fast_left=0,
                    fast_right=0,
                    line_takeover_ready=0,
                    line_takeover_cnt=0,
                    exit_boost_active=0,
                    exit_boost_cnt=0,
                    hard_outer_dynamic=0,
                    yaw_pred=0,
                    yaw_remain=0,
                    outer_scale=0,
                    takeover_valid_rows=0,
                    takeover_error=0,
                    takeover_lookahead=0,
                    takeover_trend=0,
                    box_top=vals[34],
                    box_bottom=vals[35],
                    box_left=vals[36],
                    box_right=vals[37],
                    pre_detail=vals[38],
                    turn_mark=0,
                    hard_enter_ip=0,
                    hard_enter_tick=0,
                    pre_seen_frames=0,
                )
            )
            continue

        seq = vals[0]
        pid_tick = vals[1]
        is_v6_or_v5 = record_size in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5, RECORD_SIZE_V4)
        byte_base = 23 if is_v6_or_v5 else 22
        extra_base = (67 if record_size in (RECORD_SIZE_V6, RECORD_SIZE)
                      else 55 if record_size == RECORD_SIZE_V5
                      else 46 if record_size == RECORD_SIZE_V4
                      else 45 if record_size == RECORD_SIZE_V3
                      else 0)
        is_big = record_size in (RECORD_SIZE_V6, RECORD_SIZE)

        rec = Record(
            seq=seq,
            t_ms=pid_tick * summary.pid_period_ms,
            pid_tick=pid_tick,
            base_speed=vals[2],
            speed_plan=vals[3],
            speed_out=vals[4],
            steer_out=vals[5],
            err=vals[6],
            lookahead=vals[7],
            trend=vals[8],
            yaw10=vals[9],
            ra_yaw10=vals[10],
            ra_hard_target10=vals[11],
            ra_outer_cmd=vals[12],
            yaw_rate_dps=vals[13],
            duty_left=vals[14],
            duty_right=vals[15],
            left_speed=vals[16],
            right_speed=vals[17],
            ra_timer=vals[18],
            ra_hard_cnt=vals[19],
            tf_us=vals[20],
            inter_us=vals[21],
            battery_x10=vals[22] if is_big else 0,
            valid_rows=vals[byte_base],
            ra_flag=vals[byte_base + 1],
            ra_pre=vals[byte_base + 2],
            ra_phase=vals[byte_base + 3],
            ra_dir=vals[byte_base + 4],
            ra_ip_row=vals[byte_base + 5],
            ra_slow_row=vals[byte_base + 6],
            ra_turn_row=vals[byte_base + 7],
            ra_exit_reason=vals[byte_base + 8],
            route_step=vals[byte_base + 9],
            route_flag=vals[byte_base + 10],
            route_action=vals[byte_base + 11],
            post_edge_side=vals[byte_base + 12],
            speed_reason=vals[byte_base + 13],
            line_lost=vals[byte_base + 14],
            inter_type=vals[byte_base + 15],
            sym_component=vals[byte_base + 16],
            ip_max_row=vals[byte_base + 17],
            ip_visual_row=vals[byte_base + 18] if is_big else 0,
            ip_ctrl_row=vals[byte_base + 19] if is_big else 0,
            ip_ctrl_dir=vals[byte_base + 20] if is_big else 0,
            ip_ctrl_score=vals[byte_base + 21] if is_big else 0,
            ip_ctrl_reason=vals[byte_base + 22] if is_big else 0,
            fast_ra_type=vals[byte_base + 23] if is_big or record_size == RECORD_SIZE_V5 else 0,
            fast_front=vals[byte_base + 24] if is_big or record_size == RECORD_SIZE_V5 else 0,
            fast_left=vals[byte_base + 25] if is_big or record_size == RECORD_SIZE_V5 else 0,
            fast_right=vals[byte_base + 26] if is_big or record_size == RECORD_SIZE_V5 else 0,
            line_takeover_ready=vals[byte_base + 27] if is_big else 0,
            line_takeover_cnt=vals[byte_base + 28] if is_big else 0,
            exit_boost_active=vals[byte_base + 29] if is_big else 0,
            exit_boost_cnt=vals[byte_base + 30] if is_big else 0,
            hard_outer_dynamic=vals[byte_base + 31] if is_big else 0,
            yaw_pred=vals[byte_base + 32] if is_big else 0,
            yaw_remain=vals[byte_base + 33] if is_big else 0,
            outer_scale=vals[byte_base + 34] if is_big else 0,
            takeover_valid_rows=vals[byte_base + 35] if is_big else 0,
            takeover_error=vals[byte_base + 36] if is_big else 0,
            takeover_lookahead=vals[byte_base + 37] if is_big else 0,
            takeover_trend=vals[byte_base + 38] if is_big else 0,
            box_top=vals[byte_base + 39] if is_big else vals[byte_base + 27] if record_size == RECORD_SIZE_V5 else vals[byte_base + 18],
            box_bottom=vals[byte_base + 40] if is_big else vals[byte_base + 28] if record_size == RECORD_SIZE_V5 else vals[byte_base + 19],
            box_left=vals[byte_base + 41] if is_big else vals[byte_base + 29] if record_size == RECORD_SIZE_V5 else vals[byte_base + 20],
            box_right=vals[byte_base + 42] if is_big else vals[byte_base + 30] if record_size == RECORD_SIZE_V5 else vals[byte_base + 21],
            pre_detail=vals[byte_base + 43] if is_big else vals[byte_base + 31] if record_size == RECORD_SIZE_V5 else vals[byte_base + 22],
            hard_enter_tick=vals[extra_base] if record_size in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5, RECORD_SIZE_V3) else 0,
            turn_mark=vals[extra_base + 1] if record_size in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5, RECORD_SIZE_V3) else 0,
            hard_enter_ip=vals[extra_base + 2] if record_size in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5, RECORD_SIZE_V3) else 0,
            pre_seen_frames=vals[extra_base + 3] if record_size in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V5, RECORD_SIZE_V3) else 0,
        )

        # Parse V6 suffix (16 bytes at offset 101)
        if record_size == RECORD_SIZE_V6:
            suffix = struct.unpack_from(RECORD_FMT_V6_SUFFIX, payload, off + 101)
            rec.yaw_total_progress = suffix[0]  # int16, x10
            rec.yaw_hard_progress = suffix[1]   # int16, x10
            rec.visual_exit_ready = suffix[2]
            rec.yaw_guard_active = suffix[3]
            rec.over_turn_guard = suffix[4]
            rec.line_takeover_speed_cap = suffix[5]
            rec.turn_assist_active = suffix[6]
            rec.turn_assist_weight = suffix[7]
            rec.turn_assist_found_col = suffix[8]
            rec.continuous_turn_mode = suffix[9]
            rec.exit_reason_verbose = suffix[10]
            rec.inner_min_pct = suffix[11]
            rec.outer_boost_pct = suffix[12]
            # ra_dbg_visual_stable_cnt is not in the MCU record,
            # derive from visual_exit_ready stable counting context (0 = unknown here)
            rec.visual_stable_cnt = 0
            # Save raw hex for debugging
            rec.extra_raw_101_117 = payload[off + 101:off + 117].hex()

        records.append(rec)
    return records


def unpack_live(payload: bytes, sample_period_ms: int) -> LiveRecord:
    if len(payload) == LIVE_SIZE:
        vals = struct.unpack(">H6hH2h11B", payload)
        battery_x10 = vals[7]
        duty_left = vals[8]
        duty_right = vals[9]
        byte_base = 10
        ip_max_row = vals[byte_base + 10]
    elif len(payload) == LIVE_SIZE_V3:
        vals = struct.unpack(">H8h11B", payload)
        battery_x10 = 0
        byte_base = 9
        duty_left = vals[7]
        duty_right = vals[8]
        ip_max_row = vals[byte_base + 10]
    elif len(payload) == LIVE_SIZE_V2:
        vals = struct.unpack(">H8h10B", payload)
        battery_x10 = 0
        byte_base = 9
        duty_left = vals[7]
        duty_right = vals[8]
        ip_max_row = 0
    else:
        raise ValueError(
            f"live length {len(payload)} != {LIVE_SIZE}, {LIVE_SIZE_V3}, or {LIVE_SIZE_V2}"
        )
    seq = vals[0]
    return LiveRecord(
        seq=seq,
        t_ms=seq * sample_period_ms,
        speed_plan=vals[1],
        err=vals[2],
        lookahead=vals[3],
        yaw10=vals[4],
        ra_yaw10=vals[5],
        yaw_rate_dps=vals[6],
        battery_x10=battery_x10,
        duty_left=duty_left,
        duty_right=duty_right,
        valid_rows=vals[byte_base],
        ra_flag=vals[byte_base + 1],
        ra_phase=vals[byte_base + 2],
        ra_dir=vals[byte_base + 3],
        ra_ip_row=vals[byte_base + 4],
        route_step=vals[byte_base + 5],
        post_edge_side=vals[byte_base + 6],
        speed_reason=vals[byte_base + 7],
        line_lost=vals[byte_base + 8],
        inter_type=vals[byte_base + 9],
        ip_max_row=ip_max_row,
    )


def is_turn_active(r: Record) -> bool:
    return r.ra_dir in (1, 2) and (r.ra_timer > 0 or r.ra_phase != 0 or r.ra_flag != 0)


def split_turn_segments(records: list[Record]) -> list[list[Record]]:
    segments: list[list[Record]] = []
    current: list[Record] = []
    for record in records:
        if is_turn_active(record):
            current.append(record)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def analyze(summary: Summary, records: list[Record]) -> list[str]:
    if not records:
        return ["No records received."]

    lines: list[str] = []
    duration = records[-1].t_ms - records[0].t_ms
    intervals = [
        records[i].t_ms - records[i - 1].t_ms
        for i in range(1, len(records))
        if records[i].t_ms >= records[i - 1].t_ms
    ]
    abs_err = [abs(r.err) for r in records]
    abs_la = [abs(r.lookahead) for r in records]
    lost_count = sum(1 for r in records if r.line_lost)
    pre_count = sum(1 for r in records if r.ra_pre)
    pre_dir_count = sum(1 for r in records if r.pre_detail & 0x01)
    pre_slow_count = sum(1 for r in records if r.pre_detail & 0x02)
    turn_stall_count = sum(1 for r in records if r.pre_detail & 0x40)
    wheel_stall_count = sum(1 for r in records if r.pre_detail & 0x80)
    sym_count = sum(1 for r in records if r.sym_component)
    min_rows = min(r.valid_rows for r in records)
    avg_err = statistics.fmean(abs_err)
    avg_la = statistics.fmean(abs_la)
    max_tf = max(r.tf_us for r in records)
    max_inter = max(r.inter_us for r in records)
    avg_speed_lag = statistics.fmean(
        abs(((abs(r.left_speed) + abs(r.right_speed)) // 2) - abs(r.speed_plan))
        for r in records
    )
    battery_values = [r.battery_x10 for r in records if r.battery_x10 > 0]

    if intervals:
        lines.append(
            f"records={len(records)} duration={duration}ms overflow={summary.overflow} "
            f"dt_min/max={min(intervals)}/{max(intervals)}ms"
        )
    else:
        lines.append(f"records={len(records)} duration={duration}ms overflow={summary.overflow}")
    lines.append(
        f"err avg/max={avg_err:.1f}/{max(abs_err)} "
        f"lookahead avg/max={avg_la:.1f}/{max(abs_la)} "
        f"valid_rows min={min_rows}"
    )
    lines.append(f"time max: tf={max_tf}us inter={max_inter}us, avg_speed_lag={avg_speed_lag:.1f}")
    if battery_values:
        lines.append(
            f"battery_x10 min/max={min(battery_values)}/{max(battery_values)} "
            f"({min(battery_values) / 10.0:.1f}V..{max(battery_values) / 10.0:.1f}V)"
        )
    lines.append(
        f"line_lost samples={lost_count}, ra_pre samples={pre_count}, "
        f"pre_dir={pre_dir_count}, pre_slow={pre_slow_count}, sym_component samples={sym_count}"
    )
    if turn_stall_count or wheel_stall_count:
        lines.append(f"turn_stall samples={turn_stall_count}, wheel_stall samples={wheel_stall_count}")

    segments = split_turn_segments(records)
    lines.append(f"turn_segments={len(segments)}")
    for idx, seg in enumerate(segments, 1):
        direction = "R" if seg[0].ra_dir == 1 else "L"
        max_ra_yaw = max(r.ra_yaw10 for r in seg) / 10.0
        max_yaw_rate = max(abs(r.yaw_rate_dps) for r in seg)
        max_outer_cmd = max(abs(r.ra_outer_cmd) for r in seg)
        hard_target = max(r.ra_hard_target10 for r in seg) / 10.0
        slow_row = max(r.ra_slow_row for r in seg)
        turn_row = max(r.ra_turn_row for r in seg)
        exit_reason = next((r.ra_exit_reason for r in seg if r.ra_exit_reason), 0)
        exit_name = EXIT_REASON_NAMES.get(exit_reason, f"unknown_{exit_reason}")
        hard_samples = sum(1 for r in seg if r.ra_phase == 3)
        yaw_lock_samples = sum(1 for r in seg if r.ra_phase == 4)
        recover_samples = sum(1 for r in seg if r.ra_phase == 5)
        stall_samples = sum(1 for r in seg if r.pre_detail & 0x40)
        wheel_stall_samples = sum(1 for r in seg if r.pre_detail & 0x80)
        hard_records = [r for r in seg if r.ra_phase == 3]
        hard_enter_record = next((r for r in seg if r.turn_mark & TURN_MARK_HARD_ENTER), None)
        if hard_enter_record is None and hard_records:
            hard_enter_record = hard_records[0]
        yaw_start_record = next((r for r in seg if r.turn_mark & TURN_MARK_YAW_START), None)
        duty_start_record = next((r for r in seg if r.turn_mark & TURN_MARK_DUTY_START), None)
        recover_block_samples = sum(1 for r in seg if r.turn_mark & TURN_MARK_RECOVER_BLOCK)
        hard_intervals = [
            hard_records[i].t_ms - hard_records[i - 1].t_ms
            for i in range(1, len(hard_records))
            if hard_records[i].t_ms >= hard_records[i - 1].t_ms
        ]
        start = seg[0]
        end = seg[-1]
        hard_ip = hard_enter_record.hard_enter_ip if hard_enter_record else start.ra_ip_row
        hard_t = (
            hard_enter_record.hard_enter_tick * summary.pid_period_ms
            if hard_enter_record and hard_enter_record.hard_enter_tick
            else (hard_enter_record.t_ms if hard_enter_record else start.t_ms)
        )
        yaw_start = yaw_start_record.t_ms if yaw_start_record else -1
        duty_start = duty_start_record.t_ms if duty_start_record else -1
        pre_frames = hard_enter_record.pre_seen_frames if hard_enter_record else 0
        lines.append(
            f"turn#{idx} {direction} t={start.t_ms}->{end.t_ms}ms "
            f"route={start.route_step} ip={start.ra_ip_row} slow/turn={slow_row}/{turn_row} "
            f"ra_yaw_max={max_ra_yaw:.1f} hard={hard_samples} lock={yaw_lock_samples} rec={recover_samples} "
            f"target={hard_target:.1f} outer={max_outer_cmd} "
            f"yaw_rate_max={max_yaw_rate} exit={exit_name} "
            f"stall={stall_samples}/{wheel_stall_samples} "
            f"hard_ip/t={hard_ip}/{hard_t} "
            f"yaw_start={yaw_start} duty_start={duty_start} "
            f"pre_frames={pre_frames} rec_block={recover_block_samples}"
        )
        if hard_intervals and max(hard_intervals) > summary.pid_period_ms:
            lines.append(f"  WARN turn#{idx}: hard phase still logged coarser than PID tick.")
        if hard_samples > 0 and max_ra_yaw < summary.ra_hard_yaw - 10:
            lines.append(f"  WARN turn#{idx}: yaw too small, likely exits early or turn force is not enough.")
        if max_ra_yaw > summary.ra_hard_yaw + 8:
            lines.append(f"  WARN turn#{idx}: yaw overshoot, reduce outer or yaw target / increase recovery damping.")
        if stall_samples:
            lines.append(f"  WARN turn#{idx}: hard command had weak yaw response; check traction, vacuum, current limit, or physical binding.")

    if lost_count:
        lines.append("WARN line_lost appeared during run; check exposure, threshold, or high-speed blur first.")
    if max_tf > 6000 or max_inter > 2500:
        lines.append("WARN processing time spike found; runtime debug/vision cost may affect high-speed recognition.")
    if avg_speed_lag > max(80, summary.motor_speed * 2):
        lines.append("WARN wheel speed feedback lags target; acceleration/current/traction may limit speed plan.")
    if avg_err > 18 or max(abs_err) > 45:
        lines.append("WARN straight tracking is unstable; tune Kp/Kd before raising speed more.")
    if summary.overflow:
        lines.append("WARN buffer overflow: only the newest records were kept. Increase capacity or shorten run.")

    return lines


def save_outputs(out_dir: Path, summary: Summary, records: list[Record], analysis: list[str]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = out_dir / "autotune_records.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(records[0]).keys()) if records else ["empty"])
        writer.writeheader()
        for record in records:
            writer.writerow(asdict(record))

    with (out_dir / "autotune_summary.json").open("w", encoding="utf-8") as f:
        json.dump(
            {
                "summary": asdict(summary),
                "analysis": analysis,
                "record_count": len(records),
            },
            f,
            ensure_ascii=False,
            indent=2,
        )

    with (out_dir / "analysis.txt").open("w", encoding="utf-8") as f:
        f.write("\n".join(analysis))
        f.write("\n")


def open_live_csv(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    live_path = out_dir / "live_breadcrumbs.csv"
    live_file = live_path.open("w", encoding="utf-8", newline="")
    fieldnames = list(LiveRecord.__dataclass_fields__.keys())
    writer = csv.DictWriter(live_file, fieldnames=fieldnames)
    writer.writeheader()
    live_file.flush()
    return live_file, writer, live_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive TC264 offline auto-tune logs.")
    parser.add_argument("--port", default="COM11", help="Serial port, default COM11")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate, default 115200")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout")
    parser.add_argument("--out-dir", type=Path, default=Path("autotune_runs"), help="Output root directory")
    parser.add_argument("--live-sample-ms", type=int, default=16, help="Live breadcrumb sample period before summary is known")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list:
        list_serial_ports()
        return 0

    run_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir / run_name
    summary: Summary | None = None
    records: list[Record] = []
    last_status = time.perf_counter()
    live_count = 0
    packet_counts: dict[int, int] = {}
    live_file, live_writer, live_path = open_live_csv(out_dir)

    print(f"Listening on {args.port} @ {args.baud} baud")
    print("Start this first, then run the car. Full data is dumped after motor stops.")
    print(f"Live breadcrumbs are saved immediately: {live_path}")
    print("Press Ctrl+C to stop.")

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        while True:
            try:
                packet_type, payload = read_packet(ser)
            except TimeoutError:
                now = time.perf_counter()
                if now - last_status >= 2.0:
                    state = "waiting summary" if summary is None else f"receiving {len(records)} records"
                    print(f"{state}...")
                    last_status = now
                continue
            except ValueError as exc:
                print(f"resync: {exc}", file=sys.stderr)
                continue

            if packet_type == TYPE_SUMMARY:
                packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1
                summary = unpack_summary(payload)
                records = []
                print(f"summary: count={summary.count} sample={summary.sample_period_ms}ms overflow={summary.overflow}")
                if summary.record_size not in (RECORD_SIZE_V6, RECORD_SIZE, RECORD_SIZE_V3, RECORD_SIZE_V2, LEGACY_RECORD_SIZE):
                    print(f"warning: MCU record size {summary.record_size}, PC expects {RECORD_SIZE}")
                continue

            if packet_type == TYPE_LIVE:
                packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1
                live_period = summary.sample_period_ms if summary is not None else args.live_sample_ms
                live = unpack_live(payload, live_period)
                live_writer.writerow(asdict(live))
                live_file.flush()
                live_count += 1
                if live_count % 10 == 0:
                    print(
                        f"live {live_count}: t={live.t_ms}ms "
                        f"RA={live.ra_flag}/{live.ra_phase}/{live.ra_dir} "
                        f"err={live.err} yaw={live.ra_yaw10 / 10:.1f} "
                        f"bat={live.battery_x10 / 10.0:.1f}V"
                    )
                continue

            if packet_type == TYPE_RECORD:
                packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1
                if summary is None:
                    continue
                records.extend(unpack_records(payload, summary))
                if len(records) % 120 == 0:
                    print(f"records {len(records)}/{summary.count}")
                continue

            if packet_type == TYPE_END:
                packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1
                if summary is None:
                    print("end packet without summary, ignored")
                    continue
                analysis = analyze(summary, records)
                save_outputs(out_dir, summary, records, analysis)
                live_file.close()
                print("\n".join(analysis))
                print(f"saved: {out_dir}")
                return 0

            packet_counts[packet_type] = packet_counts.get(packet_type, 0) + 1
            now = time.perf_counter()
            if now - last_status >= 2.0:
                counts = " ".join(f"{k:02X}:{v}" for k, v in sorted(packet_counts.items()))
                state = "waiting summary" if summary is None else f"receiving {len(records)} records"
                print(f"{state}... packets {counts}")
                last_status = now

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
