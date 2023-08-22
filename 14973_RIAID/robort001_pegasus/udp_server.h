#ifndef __UDP_SERVER_H__
#define __UDP_SERVER_H__

int GetTargetId(void);
void SetTargetId(int id);
void UdpSendMsg(char *msg_p);
void UdpSendTarMsg(char *tar_msg_p);

#endif // !__UDP_SERVER_H__