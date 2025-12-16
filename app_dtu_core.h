#pragma once
#include "app_dtu.h"

// 初始化/主循环
void APP_DTU_Init(void);
void APP_DTU_Handle(void);

// 定时回调/状态
void APP_DTU_Callback(void);
void APP_DTU_Status_Reset(void);
int  APP_DTU_Remote_Cnt_Sta_Get(void);

// 连接流程
uint8_t  APP_DTU_Connect_Remote_Handle(void);

// 设备电源复位
void BSP_DTU_Power_Reboot(void);