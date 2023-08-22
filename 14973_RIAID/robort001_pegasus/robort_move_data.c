#include <stdio.h>
#include <stdlib.h>
#include "iot_gpio_ex.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "iot_watchdog.h"
#include "iot_errno.h"
#include "robort_move_data.h"
#include "udp_server.h"
#define UART2_BUFF_SIZE 100
#define TRANS_SLEEP_TIME 5000 // 5000us 5ms
#define X_IN_POSITION_ERROR 8
#define Y_IN_POSITION_ERROR 8
#define YAW_IN_POSITION_ERROR 4
unsigned char uart2ReadBuff[UART2_BUFF_SIZE] = {0};
static uint8_t move_data[15];
static int x_dis_tar = 0, y_dis_tar = 0, yaw_angle_tar = 0;
static int x_dis_real = 0, y_dis_real = 0, yaw_angle_real = 0;
static uint8_t yaw_cali_state = 0;
static int servo_data[4] = {85, 40, 70, 0};

void RobortMoveInit(void)
{
    Uart2GpioInit();
    Uart2Config();
}

void Uart2GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_11);
    // 设置GPIO11的管脚复用关系为UART2_TX Set the pin reuse relationship of GPIO11 to UART2_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_11, IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_12);
    // 设置GPIO12的管脚复用关系为UART2_RX Set the pin reuse relationship of GPIO12 to UART2_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_UART2_RXD);
}

void Uart2Config(void)
{
    uint32_t ret;
    /* 初始化UART配置，波特率 115200，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART configuration, baud rate is 9600, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = IoTUartInit(HI_UART_IDX_2, &uart_attr);
    if (ret != IOT_SUCCESS)
    {
        printf("Init Uart1 Falied Error No : %d\n", ret);
        return;
    }
}

uint8_t CalBCC(uint8_t *P, int data_len)
{
    uint8_t BCC_data = 0x00;
    for (int i = 0; i < data_len; i++)
    {
        BCC_data = BCC_data ^ P[i];
    }
    return BCC_data;
}

int High8Low8ToDEC(unsigned char high, unsigned char low)
{
    int dec;
    dec = high;
    dec <<= 8;
    dec = dec | low;
    dec = dec & 0XFFFF;
    unsigned char t = high & 0X80;
    if (t == 0x80)
    {
        dec -= 1;
        dec = ~dec;
        dec = dec & 0xFFFF;
        dec = -dec;
    }
    return dec;
}

// 输入x,y,z三个轴的速度,单位是mm/s,解算并赋值给静态数组，之后与底盘控制板通讯
void RosDataCal(void)
{
    uint8_t x_high, x_low, y_high, y_low, z_high, z_low;
    x_high = x_dis_tar >> 8;
    x_low = x_dis_tar & 0XFF;
    y_high = y_dis_tar >> 8;
    y_low = y_dis_tar & 0XFF;
    z_high = yaw_angle_tar >> 8;
    z_low = yaw_angle_tar & 0XFF;
    move_data[0] = 0X7B;
    move_data[1] = yaw_cali_state;
    yaw_cali_state = 0;
    move_data[2] = 0X00;
    move_data[3] = x_high;
    move_data[4] = x_low;
    move_data[5] = y_high;
    move_data[6] = y_low;
    move_data[7] = z_high;
    move_data[8] = z_low;
    move_data[9] = servo_data[0];
    move_data[10] = servo_data[1];
    move_data[11] = servo_data[2];
    move_data[12] = servo_data[3];
    move_data[13] = CalBCC(move_data, 13);
    move_data[14] = 0X7D;
}

void InitCal(void)
{
    move_data[0] = 0X7B;
    move_data[1] = 0X00;
    move_data[2] = 0X00;
    move_data[3] = 0X00;
    move_data[4] = 0X00;
    move_data[5] = 0X00;
    move_data[6] = 0X00;
    move_data[7] = 0X00;
    move_data[8] = 0X00;
    move_data[9] = servo_data[0];
    move_data[10] = servo_data[1];
    move_data[11] = servo_data[2];
    move_data[12] = servo_data[3];
    move_data[13] = CalBCC(move_data, 13);
    move_data[14] = 0X7D;
}

void SetRobortPosition(int x, int y, int yaw)
{
    x_dis_tar = x;
    y_dis_tar = y;
    yaw_angle_tar = yaw;
    RosDataCal();
}

void SetServoData(int id, int angle)
{
    switch (id)
    {
    case 1:
        servo_data[0] = angle;
        break;
    case 2:
        servo_data[1] = angle;
        break;
    case 3:
        servo_data[2] = angle;
        break;
    case 4:
        servo_data[3] = angle;
        break;
    default:
        break;
    }
}

int GetXRealDis(void)
{
    return x_dis_real;
}
int GetYRealDis(void)
{
    return y_dis_real;
}
int GetYAWRealAng(void)
{
    return yaw_angle_real;
}

int GetXTarDis(void)
{
    return x_dis_tar;
}
int GetYTarDis(void)
{
    return y_dis_tar;
}
int GetYAWTarAng(void)
{
    return yaw_angle_tar;
}

// 0是不校准 1是逆时针校准1度 2是顺时针校准1度
void SetYawAng(uint8_t yaw_cali)
{
    yaw_cali_state = yaw_cali;
}

uint8_t GetInPositionState(void)
{
    static uint8_t in_pos_cnt = 0, in_pos_state = 0;
    if (x_dis_tar - x_dis_real < X_IN_POSITION_ERROR && x_dis_tar - x_dis_real > -X_IN_POSITION_ERROR)
    {
        if (y_dis_tar - y_dis_real < Y_IN_POSITION_ERROR && y_dis_tar - y_dis_real > -Y_IN_POSITION_ERROR)
        {
            if (yaw_angle_tar - yaw_angle_real < YAW_IN_POSITION_ERROR && yaw_angle_tar - yaw_angle_real > -YAW_IN_POSITION_ERROR)
            {
                in_pos_cnt++;
            }
            else
            {
                in_pos_cnt = 0;
                in_pos_state = 0;
            }
        }
        else
        {
            in_pos_cnt = 0;
            in_pos_state = 0;
        }
    }
    else
    {
        in_pos_cnt = 0;
        in_pos_state = 0;
    }
    if (in_pos_cnt > 3)
        in_pos_state = 1;
    else
        in_pos_state = 0;
    return in_pos_state;
}

/**
 *int类型转换为string类型
 *Int_i：	 要转换的int类型
 *String_s：转换后的string类型
 **/
void Int_To_Str(int Int_i, char *String_s)
{
    int a;
    int b = 0;                 // 用于计数
    char *ptrfing, pBuffer[5]; // 定义一个字符串数组和字符串指针，
    ptrfing = String_s;        // 内部指针指向外部指针，进行参数传递，是属于源参数传递（通过地址），
    if (Int_i < 10)            // 当整数小于10，转换为0x格式
    {
        *ptrfing++ = '0'; // 单个数字前面补0
        *ptrfing++ = Int_i + 0x30;
    }
    else
    {
        while (Int_i > 0)
        {
            a = Int_i % 10;
            Int_i = Int_i / 10;
            pBuffer[b++] = a + 0x30; // 通过计算把数字编成ASCII码形式
        }
        b--;
        for (; b >= 0; b--) // 将得到的字符串倒序
        {
            *(ptrfing++) = pBuffer[b];
        }
    }
    *ptrfing = '\0';
}

void DisMsgCal(int dis_x, int dis_y, int yaw, char *String_s)
{
    char *dis_msg;
    dis_msg = String_s;
    char msg_tem[30];
    int p_cnt = 0;
    int tem_cnt = 0;
    dis_msg[p_cnt++] = 'd';
    dis_msg[p_cnt++] = 'i';
    dis_msg[p_cnt++] = 's';
    dis_msg[p_cnt++] = ':';
    if (dis_x < 0)
    {
        dis_x = -dis_x;
        dis_msg[p_cnt++] = '-';
    }
    Int_To_Str(dis_x, msg_tem);

    while (msg_tem[tem_cnt] != '\0')
    {
        dis_msg[p_cnt++] = msg_tem[tem_cnt++];
    }
    dis_msg[p_cnt++] = ',';
    tem_cnt = 0;

    if (dis_y < 0)
    {
        dis_y = -dis_y;
        dis_msg[p_cnt++] = '-';
    }
    Int_To_Str(dis_y, msg_tem);
    while (msg_tem[tem_cnt] != '\0')
    {
        dis_msg[p_cnt++] = msg_tem[tem_cnt++];
    }
    // dis_msg[p_cnt++] = ',';
    // tem_cnt = 0;

    // Int_To_Str(dis_y, msg_tem);
    // while (msg_tem[tem_cnt] != '\0')
    // {
    //     dis_msg[p_cnt] = msg_tem[tem_cnt];
    //     p_cnt++;
    //     tem_cnt++;
    // }
    dis_msg[p_cnt++] = '\n';
    dis_msg[p_cnt++] = '\0';
    tem_cnt = 0;
    p_cnt = 0;
}

void RobortMoveDataTask(void)
{
    static uint8_t head_flag = 0; // 接收包头标志位
    static uint8_t end_flag = 0;
    static uint8_t receive_complete_flag = 0; // 接收完成标志位
    uint8_t save_num = 0;                     // 当前保存到的下标数
    uint8_t data_num = 0;                     // 接收数据位下标
    unsigned char uartReadBuff[UART2_BUFF_SIZE] = {0};
    unsigned char uartReadFiltration[48];
    uint32_t len = 0;
    RobortMoveInit();
    while (GetInitFlag() == 0)
    {
        InitCal();
        IoTUartWrite(HI_UART_IDX_2, (unsigned char *)move_data, 15);
        TaskMsleep(50);
    }
    while (1)
    {
        // IoTUartWrite(HI_UART_IDX_2, (unsigned char *)move_data, 11);
        // printf("running\n");
        TaskMsleep(50);
        RosDataCal();
        IoTUartWrite(HI_UART_IDX_2, (unsigned char *)move_data, 15);
        // for (int i = 0; i < 4; i++)
        // {
        //     printf("angle:%d,",servo_data[i]);
        // }
        // printf("\n");
        // for (int i = 0; i < 15; i++)
        // {
        //     printf("%x,",move_data[i]);
        // }
        // printf("\n");
        len = IoTUartRead(HI_UART_IDX_2, uart2ReadBuff, UART2_BUFF_SIZE);
        if (len > 0)
        {
            for (int i = 0; i < len; i++) // 判断包头
            {
                if (uart2ReadBuff[i] == 0X7B) // 包头
                {
                    head_flag = 1;
                    save_num = 0;
                    // printf("HEAD\n");
                    break;
                }
            }
            if (head_flag == 1)
            {
                for (int i = 0; i < len; i++)
                {
                    if (save_num >= 47)
                    {
                        head_flag = 0;
                        save_num = 0;
                    }
                    if (uart2ReadBuff[i] == 0X7D)
                    {
                        end_flag = 1;
                        // printf("END\n");
                        uartReadFiltration[save_num] = uart2ReadBuff[i];
                        save_num++;
                        break;
                    }
                    else
                    {
                        uartReadFiltration[save_num] = uart2ReadBuff[i];
                        save_num++;
                    }
                }
            }
            if (end_flag == 1)
            {
                // for (int i = 0; i < 48; i++)
                // {
                //     printf("Buff is:%X\n", uartReadFiltration[i]);
                // }
                if (uartReadFiltration[8] == 0x7D) // 包尾接收判断
                {
                    if (CalBCC(uartReadFiltration, 7) == uartReadFiltration[7]) // 判断BBC校验位
                    {
                        // printf("successed\n");
                        static char msg_p[50];
                        DisMsgCal(x_dis_real, y_dis_real, yaw_angle_real, msg_p);

                        receive_complete_flag = 1;

                        x_dis_real = High8Low8ToDEC(uartReadFiltration[1], uartReadFiltration[2]);
                        y_dis_real = High8Low8ToDEC(uartReadFiltration[3], uartReadFiltration[4]);
                        yaw_angle_real = High8Low8ToDEC(uartReadFiltration[5], uartReadFiltration[6]);

                        UdpSendMsg(msg_p);
                        // printf("final:%s\n", msg_p);
                        // printf("x_dis:%d\n", x_dis_real);
                        // printf("y_dis:%d\n", y_dis_real);
                        // printf("yaw:%d\n", yaw_angle_real);
                        // printf("in position state:%d\n", GetInPositionState());
                    }
                    else
                    {
                        end_flag = 0;
                        head_flag = 0;
                        // printf("BCC error");
                    }
                }
                else
                {
                    end_flag = 0;
                    head_flag = 0;
                    // printf("END error");
                }
            }
        }
    }
}

void RobortMoveDataTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();

    attr.name = "RobortMoveDataTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 5 * 1024; // 任务栈大小*1024 stack size 5*1024
    attr.priority = osPriorityAboveNormal6;

    if (osThreadNew((osThreadFunc_t)RobortMoveDataTask, NULL, &attr) == NULL)
    {
        printf("[RobortMoveDataTask] Failed to create RobortMoveDataTask!\n");
    }
}
APP_FEATURE_INIT(RobortMoveDataTaskEntry);