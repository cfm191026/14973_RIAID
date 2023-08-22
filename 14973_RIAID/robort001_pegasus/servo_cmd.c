#include <stdio.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "iot_gpio_ex.h"
#include "hi_time.h"
#include "servo_cmd.h"
#include "robort_move_data.h"

#define COUNT 10
#define FREQ_TIME 20000
static float servoAngleMem[4][3]; // 二维分别是舵机编号、舵机角度范围、舵机角度设定值
static uint16_t duty[4];
uint8_t CLAW_STATE = 1;
static hi_u64 this_fre_start_time;
static int claw_angle=110;

void servo1_Init(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_6);
    IoSetFunc(IOT_IO_NAME_GPIO_6, IOT_IO_FUNC_GPIO_6_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_6, IOT_GPIO_DIR_OUT);

    servoAngleMem[0][0] = 1;
    servoAngleMem[0][1] = 180;
    servoAngleMem[0][2] = 85;
}

void servo2_Init(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_7);
    IoSetFunc(IOT_IO_NAME_GPIO_7, IOT_IO_FUNC_GPIO_7_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_7, IOT_GPIO_DIR_OUT);

    servoAngleMem[1][0] = 2;
    servoAngleMem[1][1] = 180;
    servoAngleMem[1][2] = 40;
}

void servo3_Init(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_9);
    IoSetFunc(IOT_IO_NAME_GPIO_9, IOT_IO_FUNC_GPIO_9_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_9, IOT_GPIO_DIR_OUT);

    servoAngleMem[2][0] = 3;
    servoAngleMem[2][1] = 180;
    servoAngleMem[2][2] = 70;
}

void servo4_Init(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_10);
    IoSetFunc(IOT_IO_NAME_GPIO_10, IOT_IO_FUNC_GPIO_10_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_10, IOT_GPIO_DIR_OUT);

    servoAngleMem[3][0] = 4;
    servoAngleMem[3][1] = 180;
    servoAngleMem[3][2] = 0;
}

void claw_sensor_init(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_2);
    IoSetFunc(IOT_IO_NAME_GPIO_2, IOT_IO_FUNC_GPIO_2_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_2, IOT_GPIO_DIR_IN);
    IoSetPull(IOT_IO_NAME_GPIO_2, IOT_IO_PULL_UP);
}

void ClawOpen(void)
{
    servo_cmd(3, 125);
}

void ClawCatch(void)
{
    IotGpioValue value = IOT_GPIO_VALUE0;
    int angle = claw_angle;
    CLAW_STATE = 1;
    // while (CLAW_STATE == 1)
    // {
    //     if (angle > 40)
    //         angle -= 3;
    //     servo_cmd(3, angle);
    //     TaskMsleep(30);
    // }
    servo_cmd(3, 70);
}

void servo_cmd(int ID, int Angle)
{
    switch (ID)
    {
    case 1:
        SetServoData(1, Angle);
        break;
    case 2:
        SetServoData(2, Angle);
        break;
    case 3:
        SetServoData(3, Angle);
        claw_angle=Angle;
        break;
    case 4:
        SetServoData(4, Angle);
        break;
    default:
        break;
    }
}

void servo_control(void)
{
    hi_u64 fre_runing_time;
    fre_runing_time = hi_get_us() - this_fre_start_time;
    while ((uint64_t)fre_runing_time < FREQ_TIME)
    {
        fre_runing_time = hi_get_us() - this_fre_start_time;
    }
    printf("fre r t%d\n", (int)fre_runing_time);
    this_fre_start_time = hi_get_us();
    duty[0] = (int)(servoAngleMem[0][2] * 2000 / 180 + 500);

    duty[1] = (int)(servoAngleMem[1][2] * 2000 / 180 + 500);

    duty[2] = (int)(servoAngleMem[2][2] * 2000 / 180 + 500);

    duty[3] = (int)(servoAngleMem[3][2] * 2000 / 180 + 500);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_6, IOT_GPIO_VALUE1);
    hi_udelay(duty[0]);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_6, IOT_GPIO_VALUE0);

    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE1);
    hi_udelay(duty[1]);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);

    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_9, IOT_GPIO_VALUE1);
    hi_udelay(duty[2]);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_9, IOT_GPIO_VALUE0);

    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_10, IOT_GPIO_VALUE1);
    hi_udelay(duty[3]);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_10, IOT_GPIO_VALUE0);

    // TaskMsleep((FREQ_TIME - duty[0] - duty[1] - duty[2] - duty[3]) / 1000 - 4);
}

void Servo1a2CmdTask(void)
{
    servo1_Init();
    servo2_Init();
    while (1)
    {
        // servo_SetAngle(1, 180, servoAngleMem[0][2]);
        // servo_SetAngle(2, 180, servoAngleMem[1][2]);
        // TaskMsleep(20);
    }
}

void Servo1a2CmdTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "Servo1a2CmdTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5; // 堆栈大小为1024*2 stack size 2*1024
    attr.priority = osPriorityAboveNormal;

    if (osThreadNew((osThreadFunc_t)Servo1a2CmdTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create Hcsr04SampleTask!\n");
    }
}
// APP_FEATURE_INIT(Servo1a2CmdTaskEntry);

void Servo3a4CmdTask(void)
{
    IotGpioValue value = IOT_GPIO_VALUE1;
    claw_sensor_init();
    servo1_Init();
    servo2_Init();
    servo3_Init();
    servo4_Init();
    while (1)
    {
        // servo_SetAngle(1, 180, servoAngleMem[0][2]);
        servo_control();
        // servo_SetAngle(3, 180, servoAngleMem[2][2]);
        // servo_SetAngle(4, 180, servoAngleMem[3][2]);
        IoTGpioGetInputVal(IOT_IO_NAME_GPIO_2, &value);
        if (value == IOT_GPIO_VALUE1)
        {
            CLAW_STATE = 1;
        }
        if (value == IOT_GPIO_VALUE0)
        {
            CLAW_STATE = 0;
            // printf("ClawLimitLOW");
        }
        TaskMsleep(30);
    }
}

void Servo3a4CmdTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "Servo3a4CmdTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5;
    attr.priority = osPriorityAboveNormal;

    if (osThreadNew((osThreadFunc_t)Servo3a4CmdTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create Hcsr04SampleTask!\n");
    }
}
// APP_FEATURE_INIT(Servo3a4CmdTaskEntry);
