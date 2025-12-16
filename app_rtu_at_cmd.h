#pragma once
#include "app_rtu_at.h"

/* 多通道收发 */
app_rtu_rx_def APP_RTU_AT_Rx_Chl(uint8_t *buffer, uint16_t size);
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size);

/* AT接收解析 */
void APP_RTU_AT_Rec(uint8_t *buffer, uint16_t len);

/* 编码格式 */
void APP_RTU_AT_ENCOD_Chl(int chl);

/* IP配置接口 */
void APP_RTU_AT_Ip_Set(uint8_t ch, char* ip, uint16_t pt, uint8_t sta);
void APP_RTU_AT_Ip_Default(void);
void APP_RTU_AT_Ip_Get_All(void);
app_dtu_ip_def APP_RTU_AT_Ip_Get_Chl(uint8_t chl);

/* 查询能力 */
void APP_RTU_AT_CSQ(void);
void APP_RTU_AT_ICCID(void);

/* 主动关闭 */
void APP_RTU_AT_MIPCLOSE_Chl(uint8_t chl, uint8_t rpt);

/* 明确声明，避免隐式声明警告 */
int APP_RTU_AT_Rec_Chk_CEREG(uint8_t *buffer);