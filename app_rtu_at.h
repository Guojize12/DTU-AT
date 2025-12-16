#ifndef __APP_RTU_AT_H
#define __APP_RTU_AT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* ===== 提取自原 app_rtu_at.c 的公共宏（供 core/cmd 共用） ===== */

/* 发送指令 */
#define  RTU_AT_CMD_SEND           "AT+MIPSEND=%d,0,"

/* 通知/URC 与匹配关键字 */
#define  RTU_AT_CMD_URC            "+MIPURC"
#define  RTU_AT_CMD_URC_TCP        "\"rtcp\""      /* 接收TCP数据提示 */
#define  RTU_AT_CMD_URC_TCP_DIS    "\"disconn\""   /* TCP连接异常提示 */

/* 通用返回 */
#define  RTU_AT_OK                 "OK"
#define  RTU_AT_ERR                "ERROR:"

/* 常用AT命令关键字（用于解析） */
#define  RTU_AT_MATREADY           "+MATREADY"
#define  RTU_AT_MIPCFG             "+MIPCFG"
#define  RTU_AT_MIPSTATE           "+MIPSTATE"
#define  RTU_AT_MIPOPEN            "+MIPOPEN"
#define  RTU_AT_MIPMODE            "+MIPMODE"
#define  RTU_AT_MIPCLOSE           "+MIPCLOSE"
#define  RTU_AT_MIPSEND            "+MIPSEND"
#define  RTU_AT_MREBOOT            "+MREBOOT"
#define  RTU_AT_MCCID              "+MCCID"
#define  RTU_AT_CSQ                "+CSQ"
#define  RTU_AT_CEREG              "+CEREG"

/* 上电检测门限（保持与原实现一致） */
#define  RTU_AT_DET_NUM            380

/* ===== 共享步骤枚举（保持与原实现一致） ===== */
enum
{
    RTU_AT_STEP0 = 0,
    RTU_AT_STEP1,
    RTU_AT_STEP2,
    RTU_AT_STEP3,
    RTU_AT_STEP4,
    RTU_AT_STEP5,
    RTU_AT_STEP6,
    RTU_AT_STEP7,
    RTU_AT_STEP8,
    RTU_AT_STEP9,
    RTU_AT_STEP10,
    RTU_AT_STEP11,

    RTU_AT_STEP_MAX,
    RTU_AT_STEP_FAIL,
    RTU_AT_STEP_SUCCESS,
};

#define RTU_AT_CH_SUM  4

enum
{
    RTU_AT_RESULT_OK = 0,
    RTU_AT_RESULT_ERR,
    RTU_AT_RESULT_UNLIKE,
};

/* IP */
typedef struct
{
    uint8_t md;
    uint8_t en;
    char ip[64];
    uint16_t port;
} app_dtu_ip_def;
extern app_dtu_ip_def g_app_dtu_ip[RTU_AT_CH_SUM];

/* 卡号/信号 */
typedef struct
{
    char iccid[20];
    char sim[13];
    uint16_t signal;
    uint8_t signal_per;
    uint8_t signal_per_last;
    uint8_t  sta;
} app_rtu_sim_def;
extern app_rtu_sim_def g_app_rtu_sim;

#define  RTU_RX_LEN  256
#define  RTU_TX_LEN  256

/* 多通道 收 */
typedef struct
{
    uint8_t  rxBuf[RTU_RX_LEN];
    uint16_t rxLen;
    uint8_t  chl;
} app_rtu_rx_def;

/* 多通道 发 */
typedef struct
{
    char     txBuf[RTU_TX_LEN];
    uint16_t txNum;
    uint8_t  chl;
    uint16_t index;
} app_rtu_tx_def;

#define  RTU_TX_AT_LEN   200
#define  RTU_TX_LIST_LEN (RTU_AT_CH_SUM*3+6)

typedef struct
{
    uint8_t  tcp_close[RTU_AT_CH_SUM];
    uint8_t  tcp_cnt[RTU_AT_CH_SUM];
    uint8_t  tcp_code[RTU_AT_CH_SUM];
    uint8_t  tcp_cnt_chl;
    uint8_t  tcp_cnt_chk;
    uint8_t  step_cfg;
    uint8_t  step_next;
    uint8_t  cmd_list[RTU_TX_LIST_LEN];
    uint8_t  gps_en;

    uint32_t poweron_chk;
    uint8_t  poweron_chk_th;
    uint16_t poweron;

    uint8_t  tx_repeat;
    uint8_t  tx_buf[RTU_TX_AT_LEN];
    uint16_t tx_len;
    uint16_t net_timeout_cnt;

    uint8_t  mode;
} app_rtu_at_def;

extern app_rtu_at_def g_app_rtu_at;

/* 状态 */
int APP_RTU_AT_Chk_Cntn_Sta(void);

/* IP操作 */
void APP_RTU_AT_Ip_Set(uint8_t ch, char* ip, uint16_t pt, uint8_t sta);
void APP_RTU_AT_Ip_Get_All(void);
app_dtu_ip_def APP_RTU_AT_Ip_Get_Chl(uint8_t chl);
void APP_RTU_AT_Ip_Default(void);

/* 查询能力 */
void APP_RTU_AT_CSQ(void);
void APP_RTU_AT_ICCID(void);

/* 接收解析入口 */
void APP_RTU_AT_Rec(uint8_t *buffer, uint16_t len);

/* 主动关闭 */
void APP_RTU_AT_MIPCLOSE_Chl(uint8_t chl, uint8_t rpt);
void APP_RTU_Tcp_Cnt_Disconnect(uint8_t chl);

/* 就绪检测 */
uint8_t  APP_RTU_AT_Chk_Ready(void);

/* 多通道收发 */
app_rtu_rx_def APP_RTU_AT_Rx_Chl(uint8_t *buffer, uint16_t size);
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size);

/* 编码格式 */
void APP_RTU_AT_ENCOD_Chl(int chl);

/* 初始化 */
void APP_RTU_AT_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RTU_AT_H */

/*****END OF FILE*****/