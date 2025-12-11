#ifndef __APP_RTU_AT_H
#define __APP_RTU_AT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

#define RTU_AT_CH_SUM  4

enum
{
    RTU_AT_RESULT_OK = 0, //成功
    RTU_AT_RESULT_ERR,  //错误
    RTU_AT_RESULT_UNLIKE, //不一致
};
//IP
typedef struct
{
    uint8_t md;
    uint8_t en;
    char ip[64];
    uint16_t port;
} app_dtu_ip_def;
extern app_dtu_ip_def g_app_dtu_ip[RTU_AT_CH_SUM];

//卡号 信号
typedef struct
{
    char iccid[20];
    char sim[13];
    uint16_t signal;//信号
    uint8_t signal_per;//信号百分比
    uint8_t signal_per_last;//信号百分比
    uint8_t  sta;//更新状态,1新
} app_rtu_sim_def;
extern app_rtu_sim_def g_app_rtu_sim;

#define  RTU_RX_LEN  256
#define  RTU_TX_LEN  256

//多通道 收
typedef struct
{
    uint8_t  rxBuf[RTU_RX_LEN];
    uint16_t rxLen;
    uint8_t  chl;
} app_rtu_rx_def;

//多通道 发
typedef struct
{
    char     txBuf[RTU_TX_LEN];
    uint16_t txNum;
    uint8_t  chl;
    uint16_t index;
} app_rtu_tx_def;

#define  RTU_TX_AT_LEN  200
#define  RTU_TX_LIST_LEN  (RTU_AT_CH_SUM*3+6)
typedef struct
{
    uint8_t  tcp_close[RTU_AT_CH_SUM];//开机关闭记录
    uint8_t  tcp_cnt[RTU_AT_CH_SUM]; //tcp连接状态
    uint8_t  tcp_code[RTU_AT_CH_SUM]; //数据发送格式标志
    uint8_t  tcp_cnt_chl;//当前连接通道
    uint8_t  tcp_cnt_chk;//通道状态检测
    uint8_t  step_cfg; //指令发送步骤
    uint8_t  step_next; //指令发送步骤 下一步
    uint8_t  cmd_list[RTU_TX_LIST_LEN];//指令列表
    uint8_t  gps_en;//gps使能

    uint32_t poweron_chk;//dtu上电检测
    uint8_t  poweron_chk_th; //dtu上电检测 阈值
    uint16_t poweron;//dtu上电

    uint8_t  tx_repeat;
    uint8_t  tx_buf[RTU_TX_AT_LEN];
    uint16_t tx_len;
    uint16_t net_timeout_cnt; //掉网计数

    uint8_t  mode;//多通道使能
} app_rtu_at_def;

extern app_rtu_at_def g_app_rtu_at;

int APP_RTU_AT_Chk_Cntn_Sta(void);

void APP_RTU_AT_Ip_Set(uint8_t ch, char* ip, uint16_t pt, uint8_t sta);
void APP_RTU_AT_Ip_Get_All(void);
app_dtu_ip_def APP_RTU_AT_Ip_Get_Chl(uint8_t chl);
void APP_RTU_AT_Ip_Default(void);

void APP_RTU_AT_CSQ(void);
void APP_RTU_AT_ICCID(void);

void APP_RTU_AT_Rec(uint8_t *buffer, uint16_t len);

void APP_RTU_AT_MIPCLOSE_Chl(uint8_t chl, uint8_t rpt);
void APP_RTU_Tcp_Cnt_Disconnect(uint8_t chl);
uint8_t  APP_RTU_AT_Chk_Ready(void);

app_rtu_rx_def APP_RTU_AT_Rx_Chl(uint8_t *buffer, uint16_t size);
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size);

void APP_RTU_AT_ENCOD_Chl(int chl);

void APP_RTU_AT_Init(void);
#ifdef __cplusplus
}
#endif

#endif /* __APP_RTU_AT_H */

/*****END OF FILE****/
