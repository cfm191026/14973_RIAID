#include <stdio.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include "hi_time.h"
#include "servo_cmd.h"

// #define EN_HIGH IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_2, IOT_GPIO_VALUE1)
// #define EN_LOW IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_2, IOT_GPIO_VALUE0)

#define PUL_HIGH IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_13, IOT_GPIO_VALUE1)
#define PUL_LOW IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_13, IOT_GPIO_VALUE0)

#define DIR_HIGH IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_14, IOT_GPIO_VALUE1)
#define DIR_LOW IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_14, IOT_GPIO_VALUE0)
#define MSTEP 16 // 驱动细分数
static float ANGLE = 0;
static float TARGET_ANGLE = 0;
static uint8_t SPEED = 0;
static int8_t DIR = 0;
static uint8_t INITSTATE = 0;
static uint8_t INIT_FLAG=0;

static void StepMotorLimitLow(char *arg)
{
    (void)arg;
    INITSTATE = 1;
}

void step_motor_Init(void)
{
    // IoTGpioInit(IOT_IO_NAME_GPIO_2);
    // IoSetFunc(IOT_IO_NAME_GPIO_2, IOT_IO_FUNC_GPIO_2_GPIO);
    // IoTGpioSetDir(IOT_IO_NAME_GPIO_2, IOT_GPIO_DIR_OUT);

    IoTGpioInit(IOT_IO_NAME_GPIO_13);
    IoSetFunc(IOT_IO_NAME_GPIO_13, IOT_IO_FUNC_GPIO_13_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_13, IOT_GPIO_DIR_OUT);

    IoTGpioInit(IOT_IO_NAME_GPIO_14);
    IoSetFunc(IOT_IO_NAME_GPIO_14, IOT_IO_FUNC_GPIO_14_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_14, IOT_GPIO_DIR_OUT);

    IoTGpioInit(IOT_IO_NAME_GPIO_8);
    IoSetFunc(IOT_IO_NAME_GPIO_8, IOT_IO_FUNC_GPIO_8_GPIO);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_8, IOT_GPIO_DIR_IN);
    IoSetPull(IOT_IO_NAME_GPIO_8, IOT_IO_PULL_UP);
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_8, IOT_INT_TYPE_EDGE, IOT_GPIO_EDGE_FALL_LEVEL_LOW, StepMotorLimitLow, NULL);
}

void step_motor_enable(void)
{
    // EN_HIGH;
    ;
}

void step_motor_disable(void)
{
    // EN_LOW;
    ;
}

void put_out_one_step(void)
{
    PUL_HIGH;
    PUL_LOW;
}

void set_step_motor_dir(int8_t dir)
{
    if (dir == -1)
        DIR_HIGH;
    else if (dir == 1)
        DIR_LOW;
}

void step_motor_cmd(uint8_t speed, float targrt_angle)
{
    uint16_t cnt = 0;
    while (targrt_angle - ANGLE > 1.8 / MSTEP / 5 || ANGLE - targrt_angle > 1.8 / MSTEP / 5)
    {
        step_motor_enable();
        if (ANGLE < targrt_angle)
        {
            set_step_motor_dir(1);
            put_out_one_step();
            ANGLE += 1.8 / MSTEP / 5;
        }
        else if (ANGLE > targrt_angle)
        {
            set_step_motor_dir(-1);
            put_out_one_step();
            ANGLE -= 1.8 / MSTEP / 5;
        }
        cnt++;
        if (cnt > speed + 20)
        {
            usleep(1);
            cnt = 0;
        }
    }
}

void step_motor_init_cmd(void)
{
    uint16_t cnt = 0;
    while (INITSTATE == 0)
    {
        set_step_motor_dir(-1);
        put_out_one_step();
        if (cnt >= 15)
        {
            usleep(1);
            cnt = 0;
        }
        cnt++;
    }
    for (int i = 0; i < 1000; i++)
    {
        set_step_motor_dir(1);
        put_out_one_step();
        if (cnt >= 40)
        {
            usleep(1);
            cnt = 0;
        }
        cnt++;
    }
    TaskMsleep(50);
    INITSTATE = 0;
    while (INITSTATE == 0)
    {
        set_step_motor_dir(-1);
        put_out_one_step();
        if (cnt >= 3)
        {
            usleep(1);
            cnt = 0;
        }
        cnt++;
    }
    for (int i = 0; i < 200; i++)
    {
        set_step_motor_dir(1);
        put_out_one_step();
        if (cnt >= 10)
        {
            usleep(1);
            cnt = 0;
        }
        cnt++;
    }
    ANGLE=1;
}

void StepMotorSetData(uint8_t speed, float targrt_angle)
{
    SPEED = speed;
    TARGET_ANGLE = targrt_angle;
}

uint8_t GetInitFlag(void)
{
    return INIT_FLAG;
}

void StepMotorCmdTask(void)
{
    uint8_t pul_state = 0;
    float targrt_angle = 0;
    uint16_t cnt = 0;
    TaskMsleep(6000);
    step_motor_Init();
    ClawOpen();
    servo_cmd(4, 110);
    TaskMsleep(1000);
    servo_cmd(1, 72);
    servo_cmd(2, 70);
    TaskMsleep(1000);
    step_motor_init_cmd();
    step_motor_cmd(50,30);
    TaskMsleep(500);
    servo_cmd(4, 0);
    TaskMsleep(500);
    step_motor_cmd(50,0);
    INIT_FLAG=1;
    while (1)
    {
        step_motor_cmd(SPEED,TARGET_ANGLE);
        TaskMsleep(20);
    }
}


void StepMotorCmdTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "StepMotorCmdTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 3; // 堆栈大小为1024*5 stack size 5*1024
    attr.priority = osPriorityAboveNormal;
    if (osThreadNew((osThreadFunc_t)StepMotorCmdTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create StepMotorCmdTask\n");
    }
}
APP_FEATURE_INIT(StepMotorCmdTaskEntry);