#ifndef __STEP_MOTOR_CMD_H__
#define __STEP_MOTOR_CMD_H__
void StepMotorCmdTask(void);
void StepMotorSetData(uint8_t speed, float targrt_angle);
uint8_t GetInitFlag(void);
#endif // !__STEP_MOTOR_CMD_H__#define