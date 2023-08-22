/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * 该文件提供了基于yolov2的手部检测以及基于resnet18的手势识别，属于两个wk串行推理。
 * 该文件提供了手部检测和手势识别的模型加载、模型卸载、模型推理以及AI flag业务处理的API接口。
 * 若一帧图像中出现多个手，我们通过算法将最大手作为目标手送分类网进行推理，
 * 并将目标手标记为绿色，其他手标记为红色。
 *
 * This file provides hand detection based on yolov2 and gesture recognition based on resnet18,
 * which belongs to two wk serial inferences. This file provides API interfaces for model loading,
 * model unloading, model reasoning, and AI flag business processing for hand detection
 * and gesture recognition. If there are multiple hands in one frame of image,
 * we use the algorithm to use the largest hand as the target hand for inference,
 * and mark the target hand as green and the other hands as red.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "sample_comm_nnie.h"
#include "sample_media_ai.h"
#include "ai_infer_process.h"
#include "yolov2_hand_detect.h"
#include "vgs_img.h"
#include "ive_img.h"
#include "misc_util.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif /* End of #ifdef __cplusplus */

#define HAND_FRM_WIDTH 640
#define HAND_FRM_HEIGHT 384
#define DETECT_OBJ_MAX 32
#define RET_NUM_MAX 4
#define DRAW_RETC_THICK 2 // Draw the width of the line
#define WIDTH_LIMIT 32
#define HEIGHT_LIMIT 32
#define IMAGE_WIDTH 224 // The resolution of the model IMAGE sent to the classification is 224*224
#define IMAGE_HEIGHT 224
#define MODEL_FILE_GESTURE "/userdata/models/hand_classify/demo_inst.wk" // darknet framework wk model

    static int biggestBoxIndex;
    static IVE_IMAGE_S img;
    static DetectObjInfo objs[DETECT_OBJ_MAX] = {0};
    static RectBox boxs[DETECT_OBJ_MAX] = {0};
    static RectBox objBoxs[DETECT_OBJ_MAX] = {0};
    static RectBox remainingBoxs[DETECT_OBJ_MAX] = {0};
    static RectBox cnnBoxs[DETECT_OBJ_MAX] = {0}; // Store the results of the classification network
    static RecogNumInfo numInfo[RET_NUM_MAX] = {0};
    static IVE_IMAGE_S imgIn;
    static IVE_IMAGE_S imgDst;
    static VIDEO_FRAME_INFO_S frmIn;
    static VIDEO_FRAME_INFO_S frmDst;
    int uartFd = 0;
    static int type = 99;
    int i = 0;

#define ACCURACY 0.0001

float Derivative(float (*func)(float),float position){
	float result = func(position+ACCURACY)-func(position);
	result /= ACCURACY;
	return result;
}

float function(float x){
	return  (90*3.6*0.1)/x;	
}
 

    /*
     * 串口设置
     * Serial port settings
     */
    int Uart1Config(int fd)
    {
        struct termios newtio = {0}, oldtio = {0};
        /*
         * 获取原有串口配置
         * Get the original serial port configuration
         */
        if (tcgetattr(fd, &oldtio) != 0)
        {
            perror("SetupSerial 1");
            return -1;
        }
        (void)memset_s(&newtio, sizeof(newtio), 0, sizeof(newtio));
        /*
         * CREAD开启串行数据接收，CLOCAL打开本地连接模式
         * CREAD opens serial data reception, CLOCAL opens local connection mode
         */
        newtio.c_cflag |= CLOCAL | CREAD;

        /*
         * 设置数据位
         * Set data bit
         */
        newtio.c_cflag &= ~CSIZE;
        newtio.c_cflag |= CS8;
        /*
         * 设置奇偶校验位
         * Set parity bit
         */
        newtio.c_cflag &= ~PARENB; // 无奇偶校验
        /*
         * 设置波特率115200
         * Set baud rate 115200
         */
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);

        /*
         * 设置停止位
         * Set stop bit
         */
        newtio.c_cflag &= ~CSTOPB; /* 默认为一位停止位 */
        /*
         * 设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时
         *
         * Set the minimum characters and waiting time,
         * when there are no special requirements for receiving characters and waiting time
         */
        newtio.c_cc[VTIME] = 0; /* 非规范模式读取时的超时时间 */
        newtio.c_cc[VMIN] = 0;  /* 非规范模式读取时的最小字符数 */
        /*
         * tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来
         *
         * tcflush clears the unfinished input/output requests and data of the terminal;
         * TCIFLUSH means clearing the data being received and not reading it out
         */
        tcflush(fd, TCIFLUSH);
        if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
        {
            perror("com set error");
            return -1;
        }
        return 0;
    }

    /*
     * @berf uart 发送数据
     * @param int fd: 串口文件描述符
     * @param void *buf:发送数据buffer
     * @param int len:数据缓冲长度
     *
     * @berf uart send
     * @param int fd: uart file descriptor
     * @param void *buf:send data buf
     * @param int len:data buf len
     */
    int UartSend(int fd, char *buf, int len)
    {
        int ret = 0;
        int count = 0;
        char *sendBuf = buf;
        int sendLen = len;

        tcflush(fd, TCIFLUSH);

        while (sendLen > 0)
        {
            ret = write(fd, (char *)sendBuf, sendLen);
            if (ret < 1)
            {
                printf("write data below 1 byte % d\r\n", ret);
                break;
            }
            count += ret;
            sendLen = sendLen - ret;
        }

        return count;
    }

    /*
     * @berf uart 读取数据
     * @param int uart_fd: 串口文件描述符
     * @param void *buf: 读取数据buffer
     * @param int len: 数据缓冲区长度
     * @param int timeoutMs: 读取数据时间
     *
     * @berf uart read
     * @param int uart_fd: uart file descriptor
     * @param void *buf: read data buf
     * @param int len: data buf len
     * @param int timeoutMs: read data time
     */
    int UartRead(int fd, char *buf, int len, int timeoutMs)
    {
        int ret = 0;
        size_t rsum = 0;
        ret = 0;
        fd_set rset;
        struct timeval time;
        int timeout = timeoutMs;
        char *readBuf = buf;
        int readLen = len;

        while (rsum < readLen)
        {
            time.tv_sec = timeout / 1000;                         /* 1000:转换成秒 */
            time.tv_usec = (timeout - time.tv_sec * 1000) * 1000; /* 1000, 1000:转换为微秒 */
            FD_ZERO(&rset);
            FD_SET(fd, &rset);
            ret = select(fd + 1, &rset, NULL, NULL, &time); // 非阻塞式读取数据
            if (ret <= 0)
            {
                if (ret == 0)
                {
                    printf("time over!\r\n");
                    return -1;
                }
                if (ret == -1)
                {
                    printf("select error!\r\n");
                    continue; // 信号中断
                }
                return -1;
            }
            else
            {
                ret = read(fd, (char *)readBuf + rsum, readLen - rsum);
                if (ret < 0)
                {
                    printf("read data failed\r\n");
                    return ret;
                }
                else
                {
                    rsum += ret;
                }
            }
        }
        return rsum;
    }

    unsigned int UartOpenInit(void)
    {
        int fd;
        char *uart1 = "/dev/ttyAMA1";

        if ((fd = open(uart1, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
        {
            printf("open %s is failed", uart1);
            return -1;
        }
        else
        {
            Uart1Config(fd);
        }
        return fd;
    }

    void UartClose(int fd)
    {
        close(fd);
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

    unsigned char calBCC1(unsigned char *P, int data_len)
    {
        unsigned char BCC_data = 0x00;
        for (int i = 0; i < data_len; i++)

        {
            BCC_data = BCC_data ^ P[i];
        }
        return BCC_data;
    }

    void prInt(int x, int z, int a)
    {
        unsigned int ret = 0;
        int readLen = 0;
        unsigned char x_high, x_low, z_low;
        unsigned char uart_data[7] = {0};
        x_high = x >> 8;
        x_low = x & 0XFF;
        z_low = z & 0XFF;
        unsigned char l;
        if (a == 1)
        {

            uart_data[0] = 0X8B;
            uart_data[1] = 0XAA;
            uart_data[2] = 0XAA;
            uart_data[3] = 0XAA;
            uart_data[4] = 0XAA;
            uart_data[5] = calBCC1(uart_data, 5);
            uart_data[6] = 0X8D;
            UartSend(uartFd, uart_data, strlen(uart_data));
            printf("receive\n");
        }
        else
        {
            uart_data[0] = 0X8B;
            uart_data[1] = 0XAB;
            uart_data[2] = z_low;
            uart_data[3] = x_high;
            uart_data[4] = x_low;
            uart_data[5] = calBCC1(uart_data, 5);
            uart_data[6] = 0X8D;
            for (int i = 0; i < 8; i++)
            {
                SAMPLE_PRT("uartReadBuff%X\n", uart_data[i]);
            }
            UartSend(uartFd, uart_data, strlen(uart_data));
            printf("send\n");
        }
        printf("a2:%d\n", a);
    }

    int readForRobot(int old)
    {
        unsigned char uartReadBuff[100] = {0};
        int i = 0;
        int count = 8;
        UartRead(uartFd, uartReadBuff, 3000, 3);
        for (int i = 0; i < 100; i++)
        {
            if (uartReadBuff[i] == 0x9B)
            {
                count = i;
                break;
            }
        }

        if (uartReadBuff[count + 3] == calBCC1(uartReadBuff, 3))
        {
            int read = High8Low8ToDEC(uartReadBuff[count + 1], uartReadBuff[count + 2]);
            SAMPLE_PRT("read:%d\n", read);
            if (read != 0)
            {
                prInt(read, 99, 1);
                printf("break\n");
                return read;
            }
        }
        else
        {
            return old;
        }
    }

    /*
     * 加载手部检测和手势分类模型
     * Load hand detect and classify model
     */
    HI_S32 Yolo2HandDetectResnetClassifyLoad(uintptr_t *model)
    {
        SAMPLE_SVP_NNIE_CFG_S *self = NULL;
        HI_S32 ret;

        ret = CnnCreate(&self, MODEL_FILE_GESTURE);
        *model = ret < 0 ? 0 : (uintptr_t)self;
        HandDetectInit(); // Initialize the hand detection model
        SAMPLE_PRT("Load hand detect claasify model success\n");
        /*
         * Uart串口初始化
         * Uart open init
         */
        uartFd = UartOpenInit();
        if (uartFd < 0)
        {
            printf("uart1 open failed\r\n");
        }
        else
        {
            printf("uart1 open sxuccessed\r\n");
        }

        return ret;
    }

    HI_S32 Yolo2GoodsDetectResnetClassifyUnload(uintptr_t model)
    {
        CnnDestroy((SAMPLE_SVP_NNIE_CFG_S *)model);
        GoodsDetectExit(); // Uninitialize the hand detection model
        close(uartFd);
        SAMPLE_PRT("Unload hand detect claasify model success\n");

        return 0;
    }

    /*
     * 商品识别信息
     * Hand gesture recognition info
     */
    static int GoodsDetectFlag(const RecogNumInfo resBuf)
    {
        HI_CHAR *goodsName = NULL;
        switch (resBuf.num)
        {
        case 0u:
            goodsName = "coke";

            SAMPLE_PRT("----goods name----:%s\n", goodsName);
            return 1;

        case 1u:
            goodsName = "Sprite";
            // UartSendRead(uartFd, ForefingerGesture); // 食指手势
            SAMPLE_PRT("----goods name----:%s\n", goodsName);
            return 2;

        case 2u:
            goodsName = "Meinida";

            SAMPLE_PRT("----goods name----:%s\n", goodsName);
            return 3;

        case 3u:
            goodsName = "Rio";

            SAMPLE_PRT("----goods name----:%s\n", goodsName);
            return 4;

        case 4u:
            goodsName = "crisps";

            SAMPLE_PRT("----gesggoodsoodsture name----:%s\n", goodsName);
            return 5;

        default:
            goodsName = "goods others";

            SAMPLE_PRT("----goods name----:%s\n", goodsName);
            return 6;
            break;
        }
    }

    HI_S32 Yolo2GoodsDetectResnetClassifyCal(uintptr_t model, VIDEO_FRAME_INFO_S *srcFrm, VIDEO_FRAME_INFO_S *dstFrm)
    {
        SAMPLE_SVP_NNIE_CFG_S *self = (SAMPLE_SVP_NNIE_CFG_S *)model;
        HI_S32 resLen = 0;
        int objNum;
        int ret;
        int num = 0;

        ret = FrmToOrigImg((VIDEO_FRAME_INFO_S *)srcFrm, &img);
        SAMPLE_CHECK_EXPR_RET(ret != HI_SUCCESS, ret, "hand detect for YUV Frm to Img FAIL, ret=%#x\n", ret);

        objNum = HandDetectCal(&img, objs);

        SAMPLE_PRT("type:%d\n", type);
        type = readForRobot(type);
        usleep(100000);

        if (objNum > 0)
        {
            for (int i = 0; i < objNum; i++)
            {

                remainingBoxs[num++] = boxs[i];

                MppFrmDrawRects(dstFrm, remainingBoxs, objNum, RGB888_RED, DRAW_RETC_THICK);
            }

            /*
             * 裁剪出来的图像通过预处理送分类网进行推理
             * The cropped image is preprocessed and sent to the classification network for inference
             */

            for (int i = 0; i < objNum; i++)
            {

                cnnBoxs[i] = objs[i].box;
                RectBox *box = &objs[i].box;
                RectBoxTran(box, HAND_FRM_WIDTH, HAND_FRM_HEIGHT,
                            dstFrm->stVFrame.u32Width, dstFrm->stVFrame.u32Height);

                boxs[i] = *box;
                ret = ImgYuvCrop(&img, &imgIn, &cnnBoxs[i]);

                if ((imgIn.u32Width >= WIDTH_LIMIT) && (imgIn.u32Height >= HEIGHT_LIMIT))
                {
                    COMPRESS_MODE_E enCompressMode = srcFrm->stVFrame.enCompressMode;
                    ret = OrigImgToFrm(&imgIn, &frmIn);
                    frmIn.stVFrame.enCompressMode = enCompressMode;
                    // SAMPLE_PRT("crop u32Width = %d, img.u32Height = %d\n", imgIn.u32Width, imgIn.u32Height);
                    ret = MppFrmResize(&frmIn, &frmDst, IMAGE_WIDTH, IMAGE_HEIGHT);
                    ret = FrmToOrigImg(&frmDst, &imgDst);
                    ret = CnnCalImg(self, &imgDst, numInfo, sizeof(numInfo) / sizeof((numInfo)[0]), &resLen);
                    SAMPLE_CHECK_EXPR_RET(ret < 0, ret, "CnnCalImg FAIL, ret=%#x\n", ret);
                    HI_ASSERT(resLen <= sizeof(numInfo) / sizeof(numInfo[0]));
                    SAMPLE_PRT("type3:%d\n", type);

                    GoodsHandDetectFlag(numInfo[0]) == type)
                    {

                        SAMPLE_PRT("{min:%d max:%d}\n", box->xmin, box->xmax);
                        float x_max = (float)box->xmax;
                        float x_min = (float)box->xmin;
                        float area = (float)((box->xmax) - (box->xmin)) * ((box->ymax) - (box->ymin));
                        // 29.918
                        int z = (int)(-(area * 0.00033425) + 70);
                        int i = (int)((x_max - x_min) / 2 + x_min) - 960;
                        SAMPLE_PRT("x:%d z:%d\n", i, z);
                        // if
                        prInt(i, z, 0);
                    }
                    SAMPLE_PRT("type4:%d\n", type);

                    MppFrmDestroy(&frmDst);
                }
            }

            IveImgDestroy(&imgIn);
        }

        return ret;
    }

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
