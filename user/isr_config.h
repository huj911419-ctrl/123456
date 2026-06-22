/*********************************************************************************************************************
 * TC264 开源库 (基于官方SDK接口的第三方开源库)
 * Copyright (c) 2022 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可声明为译文版本
 * 许可声明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          isr_config
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.9.20
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

#ifndef _isr_config_h
#define _isr_config_h


/* ========================================================================
 * 【特别注意】中断优先级配置规则
 * ========================================================================
 * 1. 所有中断优先级必须设置为不同的值，不能重复！
 * 2. 优先级范围 1-255，数值越大优先级越高，0表示不开启中断
 * 3. INT_SERVICE 决定中断由谁处理（CPU0/CPU1/DMA）
 * 4. 若 INT_SERVICE 设为 IfxSrc_Tos_dma，则优先级范围限制为 0-47
 * ======================================================================== */

/* ========================================================================
 * 优先级分配方案（从高到低）：
 *   DMA通道5（摄像头DMA完成）：60    -- 最高，确保图像数据不丢失
 *   ERU通道3/7（摄像头VSYNC）：43    -- 帧同步信号
 *   ERU通道0/4（摄像头VSYNC备用）：40
 *   ERU通道1/5（ToF模块）：41
 *   CCU60_CH1（IMU更新 5ms）：31     -- IMU积分需要精确时序
 *   CCU60_CH0（PID控制 8ms）：30     -- 主控制回路
 *   CCU61通道：32-33                  -- 备用定时器
 *   UART3（电机驱动通信）：19-21      -- 电机速度反馈
 *   UART2（无线模块）：16-18
 *   UART1（摄像头串口）：13-15
 *   UART0（调试串口）：10-12          -- 最低，调试用
 * ======================================================================== */

/* ========================================================================
 * CCU6 PIT定时器中断配置
 * ========================================================================
 * CCU60 通道0：PID控制周期中断（8ms）
 *   - 由 CPU0 处理
 *   - 优先级 30
 *   - 在ISR中直接调用 line_pid_control() 执行PID计算
 *
 * CCU60 通道1：IMU数据更新中断（5ms）
 *   - 由 CPU0 处理
 *   - 优先级 31（高于PID，确保IMU时序精确）
 *   - 在ISR中调用 imu_update() 进行陀螺仪积分
 *
 * CCU61 通道0/1：备用定时器（当前未使用）
 * ======================================================================== */
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0	    /* CCU60 通道0 由 CPU0 处理 */
#define CCU6_0_CH0_ISR_PRIORITY 30	                /* CCU60 通道0 优先级=30（PID控制） */

#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0     /* CCU60 通道1 由 CPU0 处理 */
#define CCU6_0_CH1_ISR_PRIORITY 31                  /* CCU60 通道1 优先级=31（IMU更新） */

#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0     /* CCU61 通道0 由 CPU0 处理 */
#define CCU6_1_CH0_ISR_PRIORITY 32                  /* CCU61 通道0 优先级=32（备用） */

#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0     /* CCU61 通道1 由 CPU0 处理 */
#define CCU6_1_CH1_ISR_PRIORITY 33                  /* CCU61 通道1 优先级=33（备用） */



/* ========================================================================
 * GPIO外部中断（ERU）配置
 * ========================================================================
 * ERU（External Request Unit）用于处理GPIO引脚的边沿触发中断。
 * 每两个通道共享一个中断函数，通过标志位区分具体触发源。
 *
 * 通道0和通道4：摄像头VSYNC（场同步）信号检测
 *   - 用于帧同步，通知CPU一帧图像传输开始
 *
 * 通道1和通道5：ToF（激光测距）模块中断
 *   - ToF模块数据就绪时触发
 *
 * 通道2和通道6：预留（由DMA处理）
 *   - 配置为DMA触发源，不经过CPU中断
 *
 * 通道3和通道7：摄像头DMA传输完成
 *   - 通道3：P02_0 引脚，用于camera_vsync_handler()
 *   - 通道7：P15_1 引脚，预留
 * ======================================================================== */
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0	/* ERU通道0/4 由 CPU0 处理 */
#define EXTI_CH0_CH4_INT_PRIO  	40	                /* 通道0/4 优先级=40 */

#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0	/* ERU通道1/5 由 CPU0 处理 */
#define EXTI_CH1_CH5_INT_PRIO  	41	                /* 通道1/5 优先级=41 */

#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma	    /* ERU通道2/6 由 DMA 处理（不经过CPU） */
#define EXTI_CH2_CH6_INT_PRIO  	5	                /* 通道2/6 优先级=5（DMA范围0-47） */

#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0	/* ERU通道3/7 由 CPU0 处理 */
#define EXTI_CH3_CH7_INT_PRIO  	43	                /* 通道3/7 优先级=43（VSYNC同步） */


/* ========================================================================
 * DMA中断配置
 * ========================================================================
 * DMA通道5：摄像头图像数据DMA传输完成中断
 *   - 当一帧图像通过DMA传输到内存完成后触发
 *   - 由 CPU0 处理
 *   - 优先级 60（本工程最高优先级，确保图像数据完整接收）
 *   - 在ISR中调用 camera_dma_handler() 处理接收到的图像数据
 * ======================================================================== */
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu0	    /* DMA中断由 CPU0 处理 */
#define DMA_INT_PRIO  	        60	                /* DMA优先级=60（最高，确保图像完整） */


/* ========================================================================
 * UART串口中断配置
 * ========================================================================
 * UART0：调试串口
 *   - TX优先级=11，RX优先级=10，错误优先级=12
 *   - 用于发送图像数据和调试信息到PC
 *   - 接收调试命令
 *
 * UART1：摄像头串口（预留）
 *   - TX优先级=13，RX优先级=14，错误优先级=15
 *   - 用于串口摄像头数据接收
 *
 * UART2：无线模块串口
 *   - TX优先级=16，RX优先级=17，错误优先级=18
 *   - 用于无线遥控数据接收
 *
 * UART3：电机驱动通信串口
 *   - TX优先级=19，RX优先级=20，错误优先级=21
 *   - 用于向电机驱动板发送PWM占空比指令
 *   - 接收电机速度反馈数据
 * ======================================================================== */
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0	    /* UART0 由 CPU0 处理 */
#define UART0_TX_INT_PRIO       11	                /* UART0 发送优先级=11 */
#define UART0_RX_INT_PRIO       10	                /* UART0 接收优先级=10 */
#define UART0_ER_INT_PRIO       12	                /* UART0 错误优先级=12 */

#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu0     /* UART1 由 CPU0 处理 */
#define UART1_TX_INT_PRIO       13                  /* UART1 发送优先级=13 */
#define UART1_RX_INT_PRIO       14                  /* UART1 接收优先级=14 */
#define UART1_ER_INT_PRIO       15                  /* UART1 错误优先级=15 */

#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0     /* UART2 由 CPU0 处理 */
#define UART2_TX_INT_PRIO       16                  /* UART2 发送优先级=16 */
#define UART2_RX_INT_PRIO       17                  /* UART2 接收优先级=17 */
#define UART2_ER_INT_PRIO       18                  /* UART2 错误优先级=18 */

#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0     /* UART3 由 CPU0 处理 */
#define UART3_TX_INT_PRIO       19                  /* UART3 发送优先级=19 */
#define UART3_RX_INT_PRIO       20                  /* UART3 接收优先级=20 */
#define UART3_ER_INT_PRIO       21                  /* UART3 错误优先级=21 */


#endif
