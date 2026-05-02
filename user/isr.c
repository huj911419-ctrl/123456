
#include "isr_config.h"
#include "isr.h"

#include "small_driver_uart_control.h"
#include "IMU.h"
// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);
    line_pid_control();
}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    imu_update();
}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);

}
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);




    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF ģ�� INT �����ж�

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler();                           // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                  // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
#endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_uart_handler();                          // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    wireless_module_uart_handler();                 // ����ģ��ͳһ�ص�����



}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

//    gnss_uart_callback();                           // GNSS���ڻص�����
    uart_control_callback();


}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
