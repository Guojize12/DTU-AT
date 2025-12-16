#pragma once
#include "app_rtu_at.h"

/* 初始化 */
void APP_RTU_AT_Init(void);

/* 发送（底层适配） */
void APP_RTU_AT_Tx(uint8_t *buffer, uint16_t size);

/* 上电/就绪检测 */
uint8_t  APP_RTU_AT_Chk_Ready(void);
int APP_RTU_AT_Chk_Cntn_Sta(void);

/* 定时配置驱动 */
void APP_RTU_AT_Config_Handle(void);
void APP_RTU_AT_Config_Handle_Err(void);

/* 连接断开 */
void APP_RTU_Tcp_Cnt_Disconnect(uint8_t chl);

/* 暴露给其它文件使用的内部流程函数 */
void APP_RTU_AT_Rec_Cfg_Next(void);
void APP_RTU_AT_Poweron(void);
void APP_RTU_AT_MIPOPEN_Chl(uint8_t chl);
void APP_RTU_AT_ENCOD(void)
