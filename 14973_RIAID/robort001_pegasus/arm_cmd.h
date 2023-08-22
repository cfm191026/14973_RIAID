#ifndef __ARM_CMD_H__
#define __ARM_CMD_H__

void ArmCmd_InsertMode(unsigned char Speed,int X, int Y, int Z);
void arm_cmd(int16_t X, int16_t Y, int16_t Z);
void arm_set_positon(int x,int y,int z);

#endif // !__ARM_CMD_H__