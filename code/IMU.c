#include "IMU.h"
 
void imu_init(void)
{

    while(1)
    {
       if(imu660rc_init(IMU660RC_QUARTERNION_DISABLE))                            // 设置 imu660rc 以120HZ的速度产生中断触发信号
           printf("\r\n IMU660RC init error.");                                 // IMU660RC 初始化失败
       else
           break;

    }
    pit_init(CCU60_CH1, 5000); // 5000us = 5ms
}
// 在isr.c的外部中断服务中调用了 imu660rc_callback(); 完成了所有数据的采集。因此不用在单独调用 imu660rc_get_acc() 和 imu660rc_get_gyro()

void imu_get_data(void)
{
    while (1)
    {

        //在isr.c的外部中断服务中调用了 imu660rc_callback(); 完成了所有数据的采集。因此不用在单独调用 imu660rc_get_acc() 和 imu660rc_get_gyro()
        printf("\r\n IMU660RC acc data:  x=%5d, y=%5d, z=%5d\r\n", imu660rc_acc_x,  imu660rc_acc_y,  imu660rc_acc_z);
        printf("\r\n IMU660RC gyro data: x=%5d, y=%5d, z=%5d\r\n", imu660rc_gyro_x, imu660rc_gyro_y, imu660rc_gyro_z);

        system_delay_ms(1000);

    }
}
