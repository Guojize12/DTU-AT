#pragma once
#include "app_dtu.h"

// 系统配置设置/读取
void APP_DTU_Cmd_Config_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Cmd_Config_Get_Response(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Cmd_Config_Get(uint16_t cmd, uint32_t addr, uint8_t pid);

// 上传开机/SIM
void APP_DTU_SendDTUPowerOnData(void);
void APP_DTU_SendDTUSim(void);

// 上传配置整体
void APP_DTU_Send_System_config(void);

// IP管理
void APP_DTU_Cmd_Ip_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Cmd_Ip_Get(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);

// 上传频率
void APP_DTU_Cmd_Upload_Interval_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen);