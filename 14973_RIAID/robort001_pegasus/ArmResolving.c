#include "ArmResolving.h"
static double BigArmServoAngle, SmallArmServoAngle, PedestalArmServoAngle; // 分别是大臂角(0~180)、小臂角(0~180)、底座角度(0~360)

float *ServoAngleResolving(float X, float Y, float Z)
{
    static float ServoAngle[3];
    float L0, L1, L2, H; // L0是臂长投影距离 L1是大臂连接点和爪子连接端的连线 L2是底座高度减去目标高度的绝对值 H是连杆端点目标高度
    L0 = sqrt(X * X + Y * Y) - HorizontalLengthCompensationValue;
    H = Z + LengthwaysLengthCompensationValue;
    if (X > 0 & Y > 0)
        PedestalArmServoAngle = atan(Y / X) / pi * 180 + PedestalServoCompensationValue;
    else if (X<0 & Y> 0)
        PedestalArmServoAngle = 180 - atan(X / -Y) / pi * 180 + PedestalServoCompensationValue;
    else if (X < 0 & Y < 0)
        PedestalArmServoAngle = 270 - atan(-X / -Y) / pi * 180 + PedestalServoCompensationValue;
    else if (X > 0 & Y < 0)
        PedestalArmServoAngle = 360 - atan(-Y / X) / pi * 180 + PedestalServoCompensationValue;
    else if (X == 0 & Y > 0)
        PedestalArmServoAngle = 90 + PedestalServoCompensationValue;
    else if (X == 0 & Y < 0)
        PedestalArmServoAngle = 270 + PedestalServoCompensationValue;
    else if (X > 0 & Y == 0)
        PedestalArmServoAngle = PedestalServoCompensationValue;
    else if (X < 0 & Y == 0)
        PedestalArmServoAngle = 180 + PedestalServoCompensationValue;
    if (PedestalArmServoAngle > 360)
        PedestalArmServoAngle = PedestalArmServoAngle - 360;
    if (PedestalArmServoAngle < 0)
        PedestalArmServoAngle = 360 + PedestalArmServoAngle;
    ServoAngle[0] = PedestalArmServoAngle; // 底座舵机角度解算

    if (PedestalHeight > H)
    {
        L2 = PedestalHeight - H;
        L1 = sqrt(L2 * L2 + L0 * L0);
        BigArmServoAngle = atan(L0 / L2) / pi * 180 + acos((BigArmLength * BigArmLength + L1 * L1 - SmallArmLength * SmallArmLength) / (2 * BigArmLength * L1)) / pi * 180 + BigArmServoCompensationValue-90;
        SmallArmServoAngle = 180 - acos((BigArmLength * BigArmLength + SmallArmLength * SmallArmLength - L1 * L1) / (2 * BigArmLength * SmallArmLength)) / pi * 180 - BigArmServoAngle + SmallArmServoCompoensationValue;
        ServoAngle[1] = BigArmServoAngle;
        ServoAngle[2] = SmallArmServoAngle;
    }
    else if (PedestalHeight < H)
    {
        L2 = H - PedestalHeight;
        L1 = sqrt(L2 * L2 + L0 * L0);
        BigArmServoAngle = atan(L2 / L0) / pi * 180 + acos((BigArmLength * BigArmLength + L1 * L1 - SmallArmLength * SmallArmLength) / (2 * BigArmLength * L1)) / pi * 180 + BigArmServoCompensationValue ;
        SmallArmServoAngle = 180 - acos((BigArmLength * BigArmLength + SmallArmLength * SmallArmLength - L1 * L1) / (2 * BigArmLength * SmallArmLength)) / pi * 180 - BigArmServoAngle + SmallArmServoCompoensationValue;
        ServoAngle[1] = BigArmServoAngle;
        ServoAngle[2] = SmallArmServoAngle;
    }
    else if (PedestalHeight == H)
    {
        L1 = L0;
        BigArmServoAngle = acos((BigArmLength * BigArmLength + L1 * L1 - SmallArmLength * SmallArmLength) / (2 * BigArmLength * L1)) / pi * 180 + BigArmServoCompensationValue;
        SmallArmServoAngle = 180 - acos((BigArmLength * BigArmLength + SmallArmLength * SmallArmLength - L1 * L1) / (2 * BigArmLength * SmallArmLength)) / pi * 180 - BigArmServoAngle + SmallArmServoCompoensationValue;
        ServoAngle[1] = BigArmServoAngle;
        ServoAngle[2] = SmallArmServoAngle;
    }
    return ServoAngle;
}
