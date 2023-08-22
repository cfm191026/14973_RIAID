#include <stdio.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include "hi_time.h"

#include "step_motor_cmd.h"
#include "robort_move_data.h"
#include "servo_cmd.h"
#include "arm_cmd.h"
#include "taurus_uart.h"
#include "udp_server.h"

void TaskCreateTask(void)
{
    while (GetInitFlag() == 0)
    {
        TaskMsleep(100);
    }
    while (1)
    {
        SetRobortPosition(0, 0, 0);
        // while (GetInPositionState() == 0)
        // {
        //     TaskMsleep(100);
        // }
        int target_id = GetTargetId();
        while (target_id == 0)
        {
            target_id = GetTargetId();
            TaskMsleep(100);
            printf("wait wechat\n");
        }
        printf("wechat successed\n");
        SetTaurusMode(target_id);
        while (GetSendSuccFlag() == 0)
        {
            TaskMsleep(200);
        }
        SetTaurusCalibrationFlag(1);
        printf("set succeseed\n");
        while (GetTaurusCalibrationOverFlag() == 0)
        {
            TaskMsleep(200);
        }
        arm_set_positon(220, 0, 150);
        SetRobortPosition(GetXTarDis() + GetLastTaurusCaliXDis() - 150, GetYTarDis(), GetYAWTarAng());
        while (GetInPositionState() == 0)
        {
            TaskMsleep(100);
        }
        for (int i = 220; i < 280; i += 2)
        {
            arm_set_positon(i, 0, 90);
            TaskMsleep(20);
        }
        arm_set_positon(280, 0, 90);
        TaskMsleep(800);
        ClawCatch();
        TaskMsleep(500);
        arm_set_positon(290, 0, 265);
        TaskMsleep(800);
        arm_set_positon(150, 0, 265);

        SetRobortPosition(0, 0, 180);
        while (GetInPositionState() == 0)
        {
            TaskMsleep(100);
        }
        arm_set_positon(290, 0, 120);
        TaskMsleep(800);
        arm_set_positon(290, 0, 85);
        TaskMsleep(400);
        arm_set_positon(290, 0, 75);
        TaskMsleep(500);
        ClawOpen();
        TaskMsleep(300);
        arm_set_positon(150, 0, 170);
        TaskMsleep(800);
        SetTargetId(0);
    }
    // while (1)
    // {
    //     SetRobortPosition(0,400,90);
    //     while(GetInPositionState()==0)
    //     {
    //         TaskMsleep(50);
    //     }
    //     SetRobortPosition(0,-400,270);
    //     while(GetInPositionState()==0)
    //     {
    //         TaskMsleep(50);
    //     }
    // }

    while (1)
    {
        TaskMsleep(200);
    }
}

void TaskCreateTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "TaskCreateTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5; // 堆栈大小为1024*5 stack size 5*1024
    attr.priority = osPriorityNormal6;
    if (osThreadNew((osThreadFunc_t)TaskCreateTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create TaskCreateTask\n");
    }
}
APP_FEATURE_INIT(TaskCreateTaskEntry);