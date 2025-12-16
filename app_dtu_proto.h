#pragma once
#include "app_dtu.h"

/* 发送接口 */
void APP_DTU_Send_Buf(uint8_t *buffer, uint16_t size);
void APP_DTU_Send(uint8_t *buffer, uint16_t size);

/* 包头与数据校验 */
void APP_DTU_Head_Packing(uint8_t type, uint8_t *txBuf, uint16_t not_head_len, uint16_t cmd, uint8_t pid);
void APP_DTU_Remote_Head_Init(void);
int  APP_DTU_Remote_Check(void);
void APP_DTU_Rec_Handle(void);

/* 应答与解析（读/写） */
void APP_DTU_Response_Result(uint16_t cmd, uint8_t state, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Response_Hearbeat(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Parse_Read(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Parse_Write(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);

/* 心跳/时间同步 */
void APP_DTU_Send_Hearbeat(void);
void APP_DTU_GetServerTime(void);
void APP_DTU_TimeSync_Set(uint8_t *rxBuf, uint16_t rxLen);

/* GPS */
void APP_DTU_Gps_Check(uint8_t *rxBuf, uint16_t rxLen);

/* 来自 app_dtu.c 的全局变量，供协议解析使用 */
extern bsp_gps_def g_gps_date;
