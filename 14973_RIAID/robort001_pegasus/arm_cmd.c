#include "ArmResolving.h"
#include "servo_cmd.h"
#include "step_motor_cmd.h"

#include <stdio.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include "hi_time.h"

static uint8_t ServoLoedFlag = 0;
static int X = 150, Y = 0, Z = 150;

void arm_cmd(int16_t X, int16_t Y, int16_t Z)
{
    float *servo_angle;
    float yaw, step_motor_angle;
    servo_angle = ServoAngleResolving(X, Y, Z);

    // step_motor_angle = servo_angle[0] + yaw;
    // if (step_motor_angle > 360)
    //     step_motor_angle -= 360;
    // else if (step_motor_angle < 0)
    //     step_motor_angle += 360;
    // if (step_motor_angle == 360)
    //     step_motor_angle = 0;

    StepMotorSetData(15, servo_angle[0]);
    servo_cmd(1, (int)servo_angle[1]);
    servo_cmd(2, (int)servo_angle[2]);
}

void ArmCmd_InsertMode(uint8_t Speed, int X, int Y, int Z)
{
    float *ServoAngleP;
    double *ServoAngleLastTimeP;
    float ServoAngleLastTime[3];
    float ServoAngleChange[3];
    float ServoChangeMaxAngle;
    float ServoAngleCmd[3];
    ServoAngleP = ServoAngleResolving(X, Y, Z);
    // ServoAngleLastTimeP = GetServoAngleLastTime();
    for (int i = 0; i < 3; i++)
    {
        ServoAngleLastTime[i] = ServoAngleLastTimeP[i];
        if (ServoAngleP[i] - ServoAngleLastTime[i] > 0)
            ServoAngleChange[i] = ServoAngleP[i] - ServoAngleLastTime[i];
        else
            ServoAngleChange[i] = ServoAngleLastTime[i] - ServoAngleP[i];
    }
    ServoChangeMaxAngle = ServoAngleChange[0];
    if (ServoAngleChange[1] > ServoChangeMaxAngle)
        ServoChangeMaxAngle = ServoAngleChange[1];
    else if (ServoAngleChange[2] > ServoChangeMaxAngle)
        ServoChangeMaxAngle = ServoAngleChange[2];
    for (int i = 0; i < 3; i++)
    {
        ServoAngleCmd[i] = ServoAngleLastTime[i];
    }
    uint8_t SpeedCmdNum = 4; // 用来减小分辨率提升反应速度
    for (int i = 0; i < ServoChangeMaxAngle / SpeedCmdNum; i++)
    {
        if (ServoChangeMaxAngle > 0)
        {
            if (ServoAngleP[0] > ServoAngleLastTime[0])
                ServoAngleCmd[0] += ServoAngleChange[0] / ServoChangeMaxAngle * SpeedCmdNum;
            else
                ServoAngleCmd[0] -= ServoAngleChange[0] / ServoChangeMaxAngle * SpeedCmdNum;
            if (ServoAngleP[1] > ServoAngleLastTime[1])
                ServoAngleCmd[1] += ServoAngleChange[1] / ServoChangeMaxAngle * SpeedCmdNum;
            else
                ServoAngleCmd[1] -= ServoAngleChange[1] / ServoChangeMaxAngle * SpeedCmdNum;
            if (ServoAngleP[2] > ServoAngleLastTime[2])
                ServoAngleCmd[2] += ServoAngleChange[2] / ServoChangeMaxAngle * SpeedCmdNum;
            else
                ServoAngleCmd[2] -= ServoAngleChange[2] / ServoChangeMaxAngle * SpeedCmdNum;
            StepMotorSetData(80, ServoAngleCmd[0]);
            servo_cmd(1, ServoAngleCmd[1]);
            servo_cmd(2, ServoAngleCmd[2]);
            TaskMsleep(21 - Speed);
            // DelayMs(21 - Speed);
        }
    }
    arm_cmd(X, Y, Z);
}

uint8_t ArmCmd_GetServoLoadFlag(void)
{
    return ServoLoedFlag;
}

void ArmCmd_SetServoLoadFlag(uint8_t LoadState)
{
    if (LoadState == 1 | LoadState == 0)
        ServoLoedFlag = LoadState;
}

void arm_set_positon(int x, int y, int z)
{
    X = x;
    Y = y;
    Z = z;
}

void ArmCmdTask(void)
{
    while (GetInitFlag() == 0)
    {
        TaskMsleep(100);
    }
    while (1)
    {
        arm_cmd(X, Y, Z);
        TaskMsleep(200);
    }
}

void ArmCmdTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "ArmCmdTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4; // 堆栈大小为1024*5 stack size 5*1024
    attr.priority = osPriorityNormal6;

    if (osThreadNew((osThreadFunc_t)ArmCmdTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create ArmCmdTask!\n");
    }
}
APP_FEATURE_INIT(ArmCmdTaskEntry);