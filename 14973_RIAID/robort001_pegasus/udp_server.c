/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lwip/netifapi.h"

#include <hi_io.h>
#include <hi_gpio.h>
#include <hi_task.h>
#include <hi_watchdog.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "udp_config.h"

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include "robort_move_data.h"
#include "taurus_uart.h"

#define INVAILD_SOCKET (-1)
#define FREE_CPU_TIME_50MS (50)
#define INVALID_VALUE "202.202.202.202"

#define NATIVE_IP_ADDRESS "172.20.10.5" // 用户查找本地IP后需要进行修改
#define WECHAT_MSG_LIGHT_ON "_light_on"
#define WECHAT_MSG_LIGHT_OFF "_light_off"
#define DEVICE_MSG_LIGHT_ON "device_light_on"
#define DEVICE_MSG_LIGHT_OFF "device_light_off"
#define WECHAT_MSG_UNLOAD_PAGE "UnoladPage"
#define WECHAT_MSG_COLA "可乐"
#define WECHAT_MSG_SPRTIE "雪碧"
#define WECHAT_MSG_RIO "Rio"
#define WECHAT_MSG_LEFT "left"
#define WECHAT_MSG_RIGHT "right"
#define WECHAT_MSG_GO "go"
#define RECV_DATA_FLAG_OTHER (2)
#define HOST_PORT (5566)
#define DEVICE_PORT (5432)

#define UDP_RECV_LEN (255)

static int TARGET_ID = 0;
char *DIS_DATA = NULL;
char *TAR_DIS_DATA = NULL;

typedef void (*FnMsgCallBack)(hi_gpio_value val);

typedef struct FunctionCallback
{
    hi_bool stop;
    hi_u32 conLost;
    hi_u32 queueID;
    hi_u32 iotTaskID;
    FnMsgCallBack msgCallBack;
} FunctionCallback;
FunctionCallback g_gfnCallback;

int DeviceMsgCallback(FnMsgCallBack msgCallBack)
{
    g_gfnCallback.msgCallBack = msgCallBack;
    return 0;
}

int UdpTransportInit(struct sockaddr_in serAddr, struct sockaddr_in remoteAddr)
{
    int sServer = socket(AF_INET, SOCK_DGRAM, 0);
    if (sServer == INVAILD_SOCKET)
    {
        printf("create server socket failed\r\n");
        close(sServer);
    }
    // 本地主机ip和端口号
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(HOST_PORT);
    serAddr.sin_addr.s_addr = inet_addr(NATIVE_IP_ADDRESS);
    hi_sleep(500);
    if (bind(sServer, (struct sockaddr *)&serAddr, sizeof(serAddr)) == -1)
    {
        printf("bind socket failed\r\n");
        close(sServer);
    }
    // 对方ip和端口号
    remoteAddr.sin_family = AF_INET;
    remoteAddr.sin_port = htons(DEVICE_PORT);
    serAddr.sin_addr.s_addr = htons(INADDR_ANY);

    return sServer;
}

int GetTargetId(void)
{
    int target_id = TARGET_ID;
    TARGET_ID = 0;
    return target_id;
}

void SetTargetId(int id)
{
    TARGET_ID = id;
}

int sServerMsg = 0;
int len;
void UdpSendMsg(char *msg_p)
{
    DIS_DATA = msg_p;
}

void UdpSendTarMsg(char *tar_msg_p)
{
    TAR_DIS_DATA = tar_msg_p;
}

void *UdpServerDemo(const char *param)
{
    struct sockaddr_in serAddr = {0};
    struct sockaddr_in remoteAddr = {0};
    static int recvDataFlag = -1;
    char *sendData = NULL;
    int sServer = 0;

    (char *)(param);
    printf(" This Pegasus udp server demo\r\n");
    sServer = UdpTransportInit(serAddr, remoteAddr);
    sServerMsg = sServer;

    int addrLen = sizeof(remoteAddr);
    len = addrLen;
    while (1)
    {
        sendData = DIS_DATA;
        sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
        sendData = TAR_DIS_DATA;
        sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
        /* 255长度 */
        char recvData[UDP_RECV_LEN] = {0};
        int recvLen = recvfrom(sServer, recvData, UDP_RECV_LEN, 0, (struct sockaddr *)&remoteAddr, (socklen_t *)&addrLen);
        if (recvLen)
        {
            if (strstr(inet_ntoa(remoteAddr.sin_addr), INVALID_VALUE) == NULL)
            {
                // printf("A connection was received:%s\r\n", inet_ntoa(remoteAddr.sin_addr));
                // printf("data:%s\r\n", recvData);
            }
            if (strstr(recvData, WECHAT_MSG_LIGHT_OFF) != NULL)
            {
                printf("Control equipment information received:%s\r\n", recvData);
                recvDataFlag = HI_FALSE;
            }
            else if (strstr(recvData, WECHAT_MSG_LIGHT_ON) != NULL)
            {
                printf("Control equipment information received:%s\r\n", recvData);
                recvDataFlag = HI_TRUE;
            }
            else if (strstr(recvData, WECHAT_MSG_COLA) != NULL)
            {
                TARGET_ID = 1;
                printf("Control equipment information received:%s\r\n", recvData);
                sendData = "succeed";
                sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
                SetTaurusMode(TARGET_ID);
            }
            else if (strstr(recvData, WECHAT_MSG_SPRTIE) != NULL)
            {
                TARGET_ID = 2;
                printf("Control equipment information received:%s\r\n", recvData);
                sendData = "succeed";
                sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
                SetTaurusMode(TARGET_ID);
            }
            else if (strstr(recvData, WECHAT_MSG_RIO) != NULL)
            {
                TARGET_ID = 3;
                printf("Control equipment information received:%s\r\n", recvData);
                sendData = "succeed";
                sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
                SetTaurusMode(TARGET_ID);
            }
            else if (strstr(recvData, WECHAT_MSG_LEFT) != NULL)
            {
                printf("Control equipment information received:%s\r\n", recvData);
                //sendData = "succeed";
                //sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
                SetYawAng(1);
            }
            else if (strstr(recvData, WECHAT_MSG_RIGHT) != NULL)
            {
                printf("Control equipment information received:%s\r\n", recvData);
                // sendData = "succeed";
                // sendto(sServer, sendData, strlen(sendData), 0, (struct sockaddr *)&remoteAddr, addrLen);
                SetYawAng(2);
            }
            else if (strstr(recvData, WECHAT_MSG_GO) != NULL)
            {
                int x = 0, y = 0;
                unsigned char x_data_flag = 0, y_data_flag = 0;
                printf("Control equipment information received:%s\r\n", recvData);
                for (int i = 0; i < UDP_RECV_LEN; i++)
                {
                    int cal_flag = 1;
                    printf("recvData[%d]%c\n", i, recvData[i]);
                    if (recvData[i] == ':')
                    {
                        x_data_flag = 1;
                        i++;
                    }
                    if (x_data_flag == 1)
                    {
                        int symbol = 1;
                        for (int j = i; j < UDP_RECV_LEN - i; j++)
                        {
                            if (recvData[j] == ',')
                            {
                                i = j + 1;
                                break;
                            }
                            if (recvData[j] == '-')
                                symbol = -1;
                            else
                                x = x * 10 + (recvData[j] - '0') * symbol;
                            printf("x=:%d\n", x);
                        }
                        symbol = 1;
                        for (int j = i; j < UDP_RECV_LEN - i; j++)
                        {
                            if (recvData[j] == '\n')
                            {
                                cal_flag = 0;
                                break;
                            }
                            if (recvData[j] == '-')
                                symbol = -1;
                            else
                                y = y * 10 + (recvData[j] - '0') * symbol;
                            printf("y=:%d\n", y);
                        }
                    }
                    if (cal_flag == 0)
                        break;
                }
                printf("go x:%d\n", x);
                printf("go y:%d\n", y);
                SetRobortPosition(x, y, GetYAWTarAng());
            }
            else if (strstr(recvData, WECHAT_MSG_UNLOAD_PAGE) != NULL)
            {
                printf("The applet exits the current interface\r\n");
            }
            else
            {
                recvDataFlag = RECV_DATA_FLAG_OTHER;
            }
        }
        hi_sleep(FREE_CPU_TIME_50MS);
    }
    close(sServer);
    return NULL;
}

#define UDP_TASK_STACKSIZE 0x1000
#define UDP_TASK_PRIOR 27
#define UDP_TASK_NAME "UDP_demo"

static void UDPTransport(void)
{
    osThreadAttr_t attr;
#ifdef CONFIG_WIFI_AP_MODULE
    if (hi_wifi_start_softap() != 0)
    {
        printf("open softap failure\n");
        return;
    }
    printf("open softap ok\n");
#elif defined(CONFIG_WIFI_STA_MODULE)
    /* start wifi sta module */
    WifiStaModule();
#endif
    attr.name = "udp demo";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = UDP_TASK_STACKSIZE;
    attr.priority = UDP_TASK_PRIOR;

    if (osThreadNew((osThreadFunc_t)UdpServerDemo, NULL, &attr) == NULL)
    {
        printf("[UDP] Falied to create udp demo!\n");
    }
}

SYS_RUN(UDPTransport);