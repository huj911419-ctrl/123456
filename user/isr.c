#include "isr_config.h"
#include "isr.h"

#include "zf_device_small_driver_uart_control.h"
#include "IMU.h"
#include "Pid.h"

/* ========================================================================
 * 中断服务程序（ISR）实现文件
 * ========================================================================
 * 本文件包含TC264D所有外设的中断服务程序。
 * 每个ISR使用 IFX_INTERRUPT 宏定义，参数为：
 *   - 函数名
 *   - CPU编号（0或1）
 *   - 优先级（来自 isr_config.h）
 *
 * 所有ISR入口首先调用 interrupt_global_enable(0) 开启嵌套中断，
 * 允许更高优先级的中断抢占当前ISR执行。
 * ======================================================================== */


/* ========================================================================
 * PIT（周期中断定时器）中断服务程序
 * ========================================================================
 * CCU6模块提供PIT定时器，可配置为固定周期产生中断。
 * 本工程使用两个PIT通道：
 *   CCU60_CH0：PID控制回路（8ms周期，125Hz）
 *   CCU60_CH1：IMU角速度积分（5ms周期，200Hz）
 * ======================================================================== */

/**
 * @brief CCU60 通道0 PIT中断 - PID控制周期（8ms）
 *
 * 这是本工程最核心的实时控制中断。每8ms触发一次，执行：
 *   1. 转向PD控制：根据视觉误差计算转向校正量
 *   2. 速度PI控制：根据目标速度和实际速度差计算速度输出
 *   3. RA状态机：直角弯/路口的自动转弯状态机
 *   4. 速度规划：根据赛道曲率自适应调整速度
 *   5. 电机输出：将转向+速度合成后通过UART3发送给电机驱动板
 *
 * 优先级30，低于IMU更新（31），确保IMU数据时序精确。
 */
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);             /* 开启嵌套中断，允许更高优先级中断抢占 */
    pit_clear_flag(CCU60_CH0);              /* 清除中断标志位，防止重复触发 */
    line_pid_control();                     /* 执行PID控制计算和电机输出 */
}

/**
 * @brief CCU60 通道1 PIT中断 - IMU数据更新（5ms）
 *
 * 每5ms触发一次，执行IMU陀螺仪Z轴角速度的读取和积分：
 *   1. 读取陀螺仪Z轴原始角速度（度/秒）
 *   2. 减去校准时测量的零偏值
 *   3. 死区滤波（低于0.35度/秒视为噪声归零）
 *   4. 一阶低通滤波（alpha=0.65）
 *   5. 积分累加到偏航角 yaw_angle
 *   6. 归一化到 [-180, 180] 度范围
 *
 * 优先级31，高于PID控制（30），确保IMU数据时序精确。
 * yaw_angle 用于直角弯HARD阶段的角度退出判断。
 */
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    pit_clear_flag(CCU60_CH1);              /* 清除中断标志位 */
    imu_update();                           /* 执行IMU角速度积分更新 */
}

/**
 * @brief CCU61 通道0 PIT中断 - 备用定时器
 *
 * 当前未使用，预留供后续扩展。
 * 可用于添加额外的周期性任务（如传感器采样、通信轮询等）。
 */
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    pit_clear_flag(CCU61_CH0);              /* 清除中断标志位 */
}

/**
 * @brief CCU61 通道1 PIT中断 - 备用定时器
 *
 * 当前未使用，预留供后续扩展。
 */
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    pit_clear_flag(CCU61_CH1);              /* 清除中断标志位 */
}


/* ========================================================================
 * EXTI（外部中断）服务程序
 * ========================================================================
 * ERU（External Request Unit）模块提供GPIO引脚的边沿触发中断。
 * 每两个ERU通道共享一个中断函数，通过读取标志位区分触发源。
 * 主要用于摄像头帧同步和外设事件检测。
 * ======================================================================== */

/**
 * @brief ERU通道0和通道4中断 - 摄像头VSYNC信号（预留）
 *
 * 通道0：P15_4 引脚，摄像头场同步信号备用检测
 * 通道4：P15_5 引脚，摄像头场同步信号备用检测
 *
 * 当前仅清除标志位，不执行实际处理。
 * 主要的VSYNC处理在通道3（exti_ch3_ch7_isr）中完成。
 */
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))  /* 检查通道0是否触发 */
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4); /* 清除通道0中断标志 */
    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5)) /* 检查通道4是否触发 */
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5); /* 清除通道4中断标志 */
    }
}

/**
 * @brief ERU通道1和通道5中断 - ToF激光测距模块
 *
 * 通道1：P14_3 引脚，ToF模块数据就绪中断
 * 通道5：P15_8 引脚，预留
 *
 * 当ToF模块完成一次测距后，通过GPIO中断通知CPU。
 * 在中断中调用 tof_module_exti_handler() 读取测距数据。
 */
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */

    if(exti_flag_get(ERU_CH1_REQ10_P14_3)) /* 检查通道1是否触发（ToF模块） */
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3); /* 清除通道1中断标志 */
        tof_module_exti_handler();          /* 调用ToF模块中断处理函数 */
    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))  /* 检查通道5是否触发 */
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8); /* 清除通道5中断标志 */
    }
}

/**
 * @brief ERU通道3和通道7中断 - 摄像头VSYNC同步和DMA完成
 *
 * 通道3：P02_0 引脚，摄像头VSYNC（场同步）信号
 *   - 当摄像头开始输出新一帧图像时，VSYNC信号产生上升沿
 *   - 在中断中调用 camera_vsync_handler() 启动DMA接收
 *
 * 通道7：P15_1 引脚，预留
 *
 * VSYNC信号是摄像头帧同步的关键：
 *   VSYNC触发 -> 启动DMA接收 -> PCLK时钟逐字节传输图像数据 -> DMA完成中断
 */
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))  /* 检查通道3是否触发（VSYNC信号） */
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0); /* 清除通道3中断标志 */
        camera_vsync_handler();             /* 调用摄像头VSYNC处理函数，启动DMA接收 */
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1)) /* 检查通道7是否触发 */
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1); /* 清除通道7中断标志 */
    }
}


/* ========================================================================
 * DMA中断服务程序
 * ========================================================================
 * DMA（Direct Memory Access）用于高速数据传输，无需CPU逐字节搬运。
 * 摄像头图像数据通过DMA从GPIO端口自动传输到内存缓冲区。
 * ======================================================================== */

/**
 * @brief DMA通道5中断 - 摄像头图像数据接收完成
 *
 * 当一帧图像（188x120=22560字节）通过DMA传输到内存缓冲区完成后触发。
 * 在中断中调用 camera_dma_handler()：
 *   1. 设置 mt9v03x_finish_flag = 1，通知主循环新帧就绪
 *   2. 主循环检测到标志后开始视觉处理流程
 *
 * DMA传输期间CPU可执行其他任务（PID控制、IMU更新等），
 * 仅在传输完成时产生一次中断，大幅降低CPU负载。
 */
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    camera_dma_handler();                   /* 调用摄像头DMA完成处理函数 */
}


/* ========================================================================
 * UART串口中断服务程序
 * ========================================================================
 * TC264D有4个UART（UART0-3），每个UART有独立的发送、接收和错误中断。
 * 本工程UART分配：
 *   UART0：调试串口，发送图像/参数到PC，接收调试命令
 *   UART1：摄像头串口（预留，当前使用并行接口摄像头）
 *   UART2：无线模块串口，接收遥控数据
 *   UART3：电机驱动通信串口，发送PWM指令，接收速度反馈
 * ======================================================================== */

/**
 * @brief UART0 发送中断 - 调试串口发送完成
 *
 * 当UART0发送缓冲区空时触发。当前为空实现（预留）。
 * 调试数据通过轮询方式发送，不依赖发送中断。
 */
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
}

/**
 * @brief UART0 接收中断 - 调试串口接收数据
 *
 * 当UART0接收到一个字节时触发。
 * 在中断中调用 debug_interrupr_handler() 处理接收到的调试命令。
 * 仅在 DEBUG_UART_USE_INTERRUPT 宏开启时生效。
 */
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */

#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();          /* 调用调试串口中断处理函数 */
#endif
}

/**
 * @brief UART1 发送中断 - 摄像头串口发送完成（预留）
 *
 * 当前为空实现。摄像头使用并行接口，不通过UART传输图像数据。
 */
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
}

/**
 * @brief UART1 接收中断 - 摄像头串口数据接收
 *
 * 当UART1接收到数据时触发（用于串口摄像头模块）。
 * 在中断中调用 camera_uart_handler() 处理接收到的图像数据。
 */
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    camera_uart_handler();                  /* 调用摄像头串口处理函数 */
}

/**
 * @brief UART2 发送中断 - 无线模块发送完成（预留）
 *
 * 当前为空实现。无线模块主要用于接收遥控指令。
 */
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
}

/**
 * @brief UART2 接收中断 - 无线模块数据接收
 *
 * 当无线模块接收到遥控数据时触发。
 * 在中断中调用 wireless_module_uart_handler() 解析遥控指令。
 */
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    wireless_module_uart_handler();         /* 调用无线模块中断处理函数 */
}

/**
 * @brief UART3 发送中断 - 电机驱动通信发送完成（预留）
 *
 * 当前为空实现。电机驱动指令通过轮询方式发送。
 */
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
}

/**
 * @brief UART3 接收中断 - 电机驱动速度反馈接收
 *
 * 当电机驱动板返回速度数据时触发。
 * 在中断中调用 uart_control_callback() 解析速度反馈数据：
 *   - receive_left_speed_data：左电机实际速度
 *   - receive_right_speed_data：右电机实际速度
 * 速度反馈用于速度PI闭环控制。
 */
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    uart_control_callback();                /* 调用电机驱动通信回调函数 */
}


/* ========================================================================
 * UART错误中断服务程序
 * ========================================================================
 * 当UART发生帧错误、溢出错误、奇偶校验错误等异常时触发。
 * 错误处理函数 IfxAsclin_Asc_isrError() 会清除错误标志并恢复UART状态。
 * ======================================================================== */

/**
 * @brief UART0 错误中断 - 调试串口错误处理
 *
 * 当UART0发生通信错误（帧错误、溢出等）时触发。
 * 调用标准错误处理函数清除错误标志，恢复正常通信。
 */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    IfxAsclin_Asc_isrError(&uart0_handle);  /* 清除UART0错误标志 */
}

/**
 * @brief UART1 错误中断 - 摄像头串口错误处理
 */
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    IfxAsclin_Asc_isrError(&uart1_handle);  /* 清除UART1错误标志 */
}

/**
 * @brief UART2 错误中断 - 无线模块串口错误处理
 */
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    IfxAsclin_Asc_isrError(&uart2_handle);  /* 清除UART2错误标志 */
}

/**
 * @brief UART3 错误中断 - 电机驱动串口错误处理
 *
 * UART3通信错误可能导致电机失控，错误处理函数会清除错误标志
 * 并恢复正常通信，下一帧PID控制会重新发送正确的电机指令。
 */
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);             /* 开启嵌套中断 */
    IfxAsclin_Asc_isrError(&uart3_handle);  /* 清除UART3错误标志 */
}
