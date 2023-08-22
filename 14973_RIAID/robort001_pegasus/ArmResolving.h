#ifndef __ARMSOLVING_H
#define __ARMSOLVING_H
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#define HorizontalLengthCompensationValue  0//爪子中心和爪子连接端的水平距离补偿值
#define LengthwaysLengthCompensationValue  0//爪子中心和爪子连接端的竖直距离补偿值
#define PedestalServoCompensationValue 0 //底座舵机补偿角度值
#define BigArmServoCompensationValue -10//大臂舵机补偿角度值
#define SmallArmServoCompoensationValue 0 //小臂舵机补偿角度值
#define BigArmLength  200//大臂长度
#define SmallArmLength  200//小臂长度
#define PedestalHeight 110//机械臂底座高度
#define pi 3.1415926
float *ServoAngleResolving(float X, float Y, float Z);
#endif // ! __ARMSOLVING_H
