#ifndef __ROBORT_MOVE_H__
#define __ROBORT_MOVE_H__
#define PI 3.14159265
void RobortMoveDataTask(void);
void SetRobortPosition(int x,int y,int yaw);
uint8_t GetInPositionState(void);
void SetServoData(int id, int angle);
int GetXRealDis(void);
int GetYRealDis(void);
int GetYAWRealAng(void);
int GetXTarDis(void);
int GetYTarDis(void);
int GetYAWTarAng(void);
void SetYawAng(uint8_t yaw_cali);
#endif // !__ROBORT_MOVE_H__