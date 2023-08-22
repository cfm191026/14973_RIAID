#ifndef __TAURUS_UART_H__
#define __TAURUS_UART_H__

void TaurusUartTask(void);
void SetTaurusMode(int data);
uint8_t GetSendSuccFlag(void);
void SetTaurusCalibrationFlag(uint8_t flag);
uint8_t GetTaurusCalibrationOverFlag(void);
int GetLastTaurusCaliXDis(void);
void DelDisToWechat(void);


#endif // !__TAURUS_UART_H__