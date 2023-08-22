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
#include "udp_server.h"

#define UART1_BUFF_SIZE 100
#define TAURUS_ALLOW_ERROR_Y 50
#define TAURUS_ALLOW_ERROR_X 2

static int TAURUS_DATA_Y = 0;
static int TAURUS_DATA_X = 0;
static int TAURUS_MODE_DATA = 0;
static int LAST_TAURUS_X_ERR = 0;
static int DIS_X_t_TO_WECHAT = 0;
static int DIS_Y_t_TO_WECHAT = 0;
static uint8_t SET_SUCCESSED_FALG = 1;
static uint8_t TAURUS_RECEIVE_FLAG = 0;
static uint8_t SEND_SUCC_FLAG = 0;
static uint8_t TAURUS_CALIBRATION_FLAG = 0;
static uint8_t TAURUS_CALIBRATION_OVER_FLAG = 0;
static int TAKE_TASK[20][2];

typedef struct
{
    float LastP; // 上次估算协方差
    float Now_P; // 当前估算协方差
    float out;   // 卡尔曼滤波器输出
    float Kg;    // 卡尔曼增益
    float Q;     // 过程噪声协方差
    float R;     // 观测噪声协方差
} KFP;

KFP KFP_TAURUS_X = {0.02, 0, 0, 0, 0.023, 0.543};

KFP KFP_TAURUS_Y = {0.02, 0, 0, 0, 0.014, 0.543};

void Uart1GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_0);
    // 设置GPIO0的管脚复用关系为UART1_TX Set the pin reuse relationship of GPIO0 to UART1_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_0, IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_1);
    // 设置GPIO1的管脚复用关系为UART1_RX Set the pin reuse relationship of GPIO1 to UART1_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_1, IOT_IO_FUNC_GPIO_1_UART1_RXD);
}

void Uart1Config(void)
{
    uint32_t ret;
    /* 初始化UART配置，波特率 115200，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART configuration, baud rate is 9600, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr1 = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = IoTUartInit(HI_UART_IDX_1, &uart_attr1);
    if (ret != IOT_SUCCESS)
    {
        printf("Init Uart1 Falied Error No : %d\n", ret);
        return;
    }
}

uint8_t CalBCCT(uint8_t *P, int data_len)
{
    uint8_t BCC_data = 0x00;
    for (int i = 0; i < data_len; i++)
    {
        BCC_data = BCC_data ^ P[i];
    }
    return BCC_data;
}

int High8Low8ToDECT(unsigned char high, unsigned char low)
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

void SetTaurusMode(int mode_data)
{
    TAURUS_MODE_DATA = mode_data;
    SET_SUCCESSED_FALG = 0;
}

void SetTaurusTakeTask(int id, int num)
{
}

uint8_t GetSendSuccFlag(void)
{
    if (SEND_SUCC_FLAG == 1)
    {
        SEND_SUCC_FLAG = 0;
        return 1;
    }
    else
        return 0;
}

uint8_t GetTaurusCalibrationOverFlag(void)
{
    return TAURUS_CALIBRATION_OVER_FLAG;
}

void SetTaurusCalibrationFlag(uint8_t flag)
{
    if (flag == 1)
    {
        TAURUS_CALIBRATION_OVER_FLAG = 0;
        TAURUS_CALIBRATION_FLAG = 1;
    }
    else
        TAURUS_CALIBRATION_FLAG = 0;
}

int GetLastTaurusCaliXDis(void)
{
    return LAST_TAURUS_X_ERR * 10;
}

void DelDisToWechat(void)
{
    DIS_X_t_TO_WECHAT = 0;
    DIS_Y_t_TO_WECHAT = 0;
}

void Int_To_Str_T(int Int_i2, char *String_s2)
{
    int a;
    int b = 0;                 // 用于计数
    char *ptrfing, pBuffer[5]; // 定义一个字符串数组和字符串指针，
    ptrfing = String_s2;       // 内部指针指向外部指针，进行参数传递，是属于源参数传递（通过地址），
    if (Int_i2 < 10)           // 当整数小于10，转换为0x格式
    {
        *ptrfing++ = '0'; // 单个数字前面补0
        *ptrfing++ = Int_i2 + 0x30;
    }
    else
    {
        while (Int_i2 > 0)
        {
            a = Int_i2 % 10;
            Int_i2 = Int_i2 / 10;
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

void TarDisMsgCal(int dis_x_t, int dis_y_t, char *String_s_t)
{
    char *dis_msg_t;
    dis_msg_t = String_s_t;
    char msg_tem_t[30];
    int p_cnt = 0;
    int tem_cnt = 0;
    dis_msg_t[p_cnt++] = 't';
    dis_msg_t[p_cnt++] = 'a';
    dis_msg_t[p_cnt++] = 'r';
    dis_msg_t[p_cnt++] = ':';
    if (dis_x_t < 0)
    {
        dis_x_t = -dis_x_t;
        dis_msg_t[p_cnt++] = '-';
    }
    Int_To_Str_T(dis_x_t, msg_tem_t);

    while (msg_tem_t[tem_cnt] != '\0')
    {
        dis_msg_t[p_cnt++] = msg_tem_t[tem_cnt++];
    }
    dis_msg_t[p_cnt++] = ',';
    tem_cnt = 0;
    if (dis_y_t < 0)
    {
        dis_y_t = -dis_y_t;
        dis_msg_t[p_cnt++] = '-';
    }
    Int_To_Str_T(dis_y_t, msg_tem_t);
    while (msg_tem_t[tem_cnt] != '\0')
    {
        dis_msg_t[p_cnt++] = msg_tem_t[tem_cnt++];
    }
    dis_msg_t[p_cnt++] = '\n';
    dis_msg_t[p_cnt++] = '\0';
    tem_cnt = 0;
    p_cnt = 0;
    int cnt = 0;
}

float KalmanFilter(KFP *kfp, float input)
{
    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}

void KalmanTaurusXInit(KFP *kfp, int init_data_x)
{
    kfp->LastP = init_data_x;
    kfp->Now_P = init_data_x;
}

void KalmanTaurusYInit(KFP *kfp, int init_data_y)
{
    kfp->LastP = init_data_y;
    kfp->Now_P = init_data_y;
}

void TaurusCalibration(uint8_t cali_flag)
{
    static int cali_cnt = 0;
    if (cali_cnt == 0)
    {
        KalmanTaurusXInit(&KFP_TAURUS_X, TAURUS_DATA_X);
        KalmanTaurusYInit(&KFP_TAURUS_Y, TAURUS_DATA_Y);
        cali_cnt++;
    }
    if (cali_flag == 1)
    {
        static uint8_t calibration_cnt = 0;
        static char msg_p_t[50];

        int x_dis_out, y_dis_out;
        int data_x, data_y, data_x_tar;
        data_x = (int)KalmanFilter(&KFP_TAURUS_X, TAURUS_DATA_X);
        data_y = (int)KalmanFilter(&KFP_TAURUS_Y, TAURUS_DATA_Y);
        DIS_X_t_TO_WECHAT = data_x * 10 + GetXRealDis();
        DIS_Y_t_TO_WECHAT = -data_y / 50 + GetYRealDis();

        TarDisMsgCal(DIS_X_t_TO_WECHAT, DIS_Y_t_TO_WECHAT, msg_p_t);
        UdpSendTarMsg(msg_p_t);

        // printf("data_x=%d\n", data_x);
        // printf("data_y=%d\n", data_y / 10);
        printf("data:%d,%d,%d,%d\n", TAURUS_DATA_X, data_x, TAURUS_DATA_Y, data_y);
        data_x_tar = data_x - 42;

        if ((data_x_tar < TAURUS_ALLOW_ERROR_X && data_x_tar > -TAURUS_ALLOW_ERROR_X) && (data_y < TAURUS_ALLOW_ERROR_Y && data_y > -TAURUS_ALLOW_ERROR_Y))
        {
            calibration_cnt++;
            if (calibration_cnt >= 6)
            {
                cali_cnt = 0;
                TAURUS_CALIBRATION_OVER_FLAG = 1;
                TAURUS_CALIBRATION_FLAG = 0;
                calibration_cnt = 0;
                LAST_TAURUS_X_ERR = data_x;
                TarDisMsgCal(0, 0, msg_p_t);
                UdpSendTarMsg(msg_p_t);
            }
        }
        else
        {
            calibration_cnt = 0;
        }

        if (data_x_tar < TAURUS_ALLOW_ERROR_X && data_x_tar > -TAURUS_ALLOW_ERROR_X)
        {
            x_dis_out = GetXTarDis();
        }
        else
        {
            if (data_x_tar < 8 && data_x_tar > 0)
            {
                x_dis_out = 2.6 * data_x_tar + GetXRealDis();
            }
            else if (data_x_tar < 25 && data_x_tar > 8)
            {
                x_dis_out = 3.2 * data_x_tar + GetXRealDis();
            }
            else
            {
                x_dis_out = 4.6 * data_x_tar + GetXRealDis();
            }
        }
        if (data_y < TAURUS_ALLOW_ERROR_Y && data_y > -TAURUS_ALLOW_ERROR_Y)
        {
            y_dis_out = GetYTarDis();
        }
        else
        {
            if (data_y < 320 && data_y > -320)
            {
                y_dis_out = -1 * 0.17 * data_y + GetYRealDis();
            }
            else if (data_y < 560 && data_y > -560)
            {
                y_dis_out = -1 * 0.25 * data_y + GetYRealDis();
            }
            else
            {
                y_dis_out = -1 * 0.3 * data_y + GetYRealDis();
            }
        }
        SetRobortPosition(x_dis_out, y_dis_out, GetYAWTarAng());
    }
}

void TaurusUartTask(void)
{
    Uart1GpioInit();
    Uart1Config();
    unsigned char uart1ReadBuff[UART1_BUFF_SIZE] = {0};
    static uint8_t head_flag = 0; // 接收包头标志位
    static uint8_t end_flag = 0;
    static uint8_t receive_complete_flag = 0; // 接收完成标志位
    uint8_t save_num = 0;                     // 当前保存到的下标数
    uint8_t data_num = 0;                     // 接收数据位下标
    unsigned char uartReadFiltration[48];
    uint32_t len = 0;
    // 对UART1的一些初始化 Some initialization of UART1
    Uart1GpioInit();
    // 对UART1参数的一些配置 Some configurations of UART1 parameters
    Uart1Config();
    while (1)
    {
        TaskMsleep(100);
        len = IoTUartRead(HI_UART_IDX_1, uart1ReadBuff, UART1_BUFF_SIZE);
        // TaskMsleep(20);
        if (SET_SUCCESSED_FALG == 0) // TAURUS数据设置未成功
        {
            unsigned char data_tem[5];
            unsigned char data_high, data_low;
            data_high = TAURUS_MODE_DATA >> 8;  // 取出高八位
            data_low = TAURUS_MODE_DATA & 0XFF; // 取出低八位
            data_tem[0] = 0x9B;
            data_tem[1] = data_high;
            data_tem[2] = data_low;
            data_tem[3] = CalBCCT(data_tem, 3);
            data_tem[4] = 0x9D;
            IoTUartWrite(HI_UART_IDX_1, (unsigned char *)data_tem, 5);
        }
        if (SEND_SUCC_FLAG == 1)
            SET_SUCCESSED_FALG = 1;
        if (len > 0)
        {
            for (int i = 0; i < len; i++) // 判断包头
            {
                if (uart1ReadBuff[i] == 0X8B) // 包头
                {
                    head_flag = 1;
                    save_num = 0;
                    break;
                }
            }
            // for (int i = 0; i < len; i++) // 判断包头
            // {
            //     printf("%X\n", uart1ReadBuff[i]);
            // }
            if (head_flag == 1)
            {
                for (int i = 0; i < len; i++)
                {
                    if (save_num >= UART1_BUFF_SIZE - 1)
                    {
                        head_flag = 0;
                        save_num = 0;
                    }
                    if (uart1ReadBuff[i] == 0X8D)
                    {
                        end_flag = 1;
                        // printf("END\n");
                        uartReadFiltration[save_num] = uart1ReadBuff[i];
                        save_num++;
                        break;
                    }
                    else
                    {
                        uartReadFiltration[save_num] = uart1ReadBuff[i];
                        save_num++;
                    }
                }
                // if (end_flag == 1 && uartReadFiltration[1] == 0xAB)
                // {
                //     if (CalBCCT(uartReadFiltration, 5) == uartReadFiltration[5]) // 判断BBC校验位
                //     {
                //         TAURUS_DATA_X = uartReadFiltration[2];
                //         TAURUS_DATA_Y = High8Low8ToDECT(uartReadFiltration[3], uartReadFiltration[4]);
                //         TAURUS_RECEIVE_FLAG = 1;
                //     }
                //     end_flag=0;
                //     head_flag=0;
                // }
            }
            if (end_flag == 1)
            {
                if (uartReadFiltration[6] == 0x8D) // 包尾接收判断
                {
                    if (CalBCCT(uartReadFiltration, 5) == uartReadFiltration[5]) // 判断BBC校验位
                    {
                        if (uartReadFiltration[1] == 0xAA)
                        {
                            SEND_SUCC_FLAG = 1;
                            printf("SEND_SUCC_FLAG = 1\n");
                        }
                        else if (uartReadFiltration[1] == 0xAB && SET_SUCCESSED_FALG == 1)
                        {
                            TAURUS_DATA_X = uartReadFiltration[2];
                            TAURUS_DATA_Y = High8Low8ToDECT(uartReadFiltration[3], uartReadFiltration[4]);
                            TAURUS_RECEIVE_FLAG = 1;
                            TaurusCalibration(TAURUS_CALIBRATION_FLAG);
                        }
                        // printf("successed\n");
                        // for (int i = 0; i < 7; i++)
                        // {
                        //     printf("%X\n", uartReadFiltration[i]);
                        //     //printf("%D\n", uartReadFiltration[i]);
                        // }
                        // printf("ud_x:%d\n", TAURUS_DATA_X);
                        // printf("ud_y:%d\n", TAURUS_DATA_Y);
                        // printf("complete over\n");
                    }
                    else
                    {
                        end_flag = 0;
                        head_flag = 0;
                    }
                }
                else
                {
                    end_flag = 0;
                    head_flag = 0;
                }
            }
        }
    }
}

void TaurusUartTaskEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "TaurusUartTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 8; // 堆栈大小为1024*5 stack size 5*1024
    attr.priority = osPriorityAboveNormal6;
    if (osThreadNew((osThreadFunc_t)TaurusUartTask, NULL, &attr) == NULL)
    {
        printf("[GA12N205Task] Failed to create TaurusUartTask\n");
    }
}
APP_FEATURE_INIT(TaurusUartTaskEntry);