#include "app_rtu_at_core.h"
#include "app_rtu_at_cmd.h"
#include "app_config.h"

app_rtu_at_def g_app_rtu_at = { .poweron_chk_th = 35, };
static Timer g_rtu_at_timer = {0};
static Timer g_rtu_at_config_timer = {0};
static Timer g_rtu_at_timer_err = {0};

void APP_RTU_AT_Tx(uint8_t *buffer, uint16_t size)
{
    APP_DTU_Send_Buf(buffer, size);
}

int APP_RTU_AT_Chk_Cntn_Sta(void)
{
    return g_dtu_cmd.net_status;
}

/* 内部：就绪检测与驱动 */
static void APP_RTU_AT_timer(void);
static void APP_RTU_AT_CEREG(void);
static void APP_RTU_AT_MIPOPEN(void);
static void APP_RTU_AT_MIPCLOSE(void);
static void APP_RTU_AT_Tx_To(uint8_t *buffer, uint16_t size);
static void APP_RTU_AT_MIPSTATE(uint8_t chl, uint8_t rpt);
void APP_RTU_AT_RESET_Cfg(void);
void APP_RTU_AT_RESET(uint8_t sta);
void APP_RTU_AT(void);

/* 对外可见：推进下一步 */
void APP_RTU_AT_Rec_Cfg_Next(void)
{
    if (g_app_rtu_at.poweron < 2 || g_app_rtu_at.step_next < RTU_TX_LIST_LEN - 1)
    {
        g_app_rtu_at.step_cfg = g_app_rtu_at.cmd_list[++g_app_rtu_at.step_next];
    }
}

/* 对外可见：上电流程 */
void APP_RTU_AT_Poweron(void)
{
    uint8_t num = 0;
    APP_RTU_AT_RESET_Cfg();
    g_app_rtu_at.poweron = 1; // rtu上电
    g_app_rtu_at.poweron_chk = RTU_AT_DET_NUM + 3;

    g_app_rtu_at.step_cfg = RTU_AT_STEP10; // 查询网络注册状态
    g_app_rtu_at.step_next = 0;

    memset(g_app_rtu_at.tcp_close, 0, RTU_AT_CH_SUM);
    memset(g_app_rtu_at.tcp_cnt, 1, RTU_AT_CH_SUM);
    memset(g_app_rtu_at.cmd_list, 0, 16);

    APP_RTU_AT_Ip_Get_All();
    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP10;
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP11; // 数据发送格式
    }

    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_dtu_ip[i].en > 0)
        {
            g_app_rtu_at.tcp_close[i] = 1;
            g_app_rtu_at.tcp_cnt[i] = 0;
            g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP9; // 关闭TCP/IP连接
            g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP4; // 建立TCP/IP连接
        }
    }

    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP6; // 查询设备ICCID号
    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP7; // 查询设备网络信号强度

    BSP_TIMER_Start(&g_rtu_at_timer_err); // 配置超时
}

/* 对外可见：指定通道重连 */
void APP_RTU_AT_MIPOPEN_Chl(uint8_t chl)
{
    g_app_rtu_at.tcp_cnt_chl = chl;
    char buf[RTU_TX_AT_LEN] = {0};
    sprintf(buf, "AT+MIPOPEN=%d,\"TCP\",%s,%d\r\n", chl, g_app_dtu_ip[chl].ip, g_app_dtu_ip[chl].port);
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
    LOGT("Reconnecting tcp[%d]:\"%s\":%d\n", chl, g_app_dtu_ip[chl].ip, g_app_dtu_ip[chl].port);
}

/* 由 cmd 调用：强制重新配置编码 */
void APP_RTU_AT_ENCOD(void)
{
    char buf[32] = {0};
    int ret = -1;
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_rtu_at.tcp_code[i] == 0)
        {
            g_app_rtu_at.tcp_code[i] = 1;
            sprintf(buf, "AT+MIPCFG=\"encoding\",%d,1,0\r\n", i);
            APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
            ret = 0;
            break;
        }
    }
    if (ret < 0)
    {
        LOGT("err: encod\n");
        APP_RTU_AT_Rec_Cfg_Next();
    }
}

/* ===== 以下为原文件内部工具函数 ===== */

static void APP_RTU_AT_timer(void)
{
    if (g_app_rtu_at.tx_repeat > 0)
    {
        if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP4
            || g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP9)
        {
            LOGT("power on ctu..");
            APP_RTU_AT_Rec_Cfg_Next();
        }
        else
        {
            g_app_rtu_at.tx_repeat--;
            APP_RTU_AT_Tx(g_app_rtu_at.tx_buf, g_app_rtu_at.tx_len);
            BSP_TIMER_Start(&g_rtu_at_timer);
        }
    }
    else
    {
        g_app_rtu_at.step_cfg = g_app_rtu_at.cmd_list[++g_app_rtu_at.step_next];
        if (g_app_rtu_at.step_cfg == 0)
        {
            g_app_rtu_at.step_cfg = RTU_AT_STEP_FAIL;
        }
    }
}

static void APP_RTU_AT_Tx_To(uint8_t *buffer, uint16_t size)
{
    BSP_TIMER_Stop(&g_rtu_at_timer);
    BSP_TIMER_Start(&g_rtu_at_timer);
    g_app_rtu_at.tx_repeat = 3;

    g_app_rtu_at.tx_len = size;
    memset(g_app_rtu_at.tx_buf, 0, RTU_TX_AT_LEN);
    memcpy(g_app_rtu_at.tx_buf, buffer, size);

    APP_RTU_AT_Tx(g_app_rtu_at.tx_buf, size);
    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOGT("rtu w: %s\n", buffer);
    }
}

void APP_RTU_AT_RESET_Cfg(void)
{
    for (int i = 0; i < RTU_AT_CH_SUM; i++) { g_app_rtu_at.tcp_code[i] = 0; }
}

void APP_RTU_AT_RESET(uint8_t sta)
{
    char buf[16] = "AT+MREBOOT\r\n";
    if (sta == 0) { APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf)); }
    else { APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf)); }
}

void APP_RTU_AT(void)
{
    char buf[6] = "AT\r\n";
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
}

static void APP_RTU_AT_MIPSTATE(uint8_t chl, uint8_t rpt)
{
    char buf[32] = {0};
    sprintf(buf, "AT+MIPSTATE=%d\r\n", chl);
    if (rpt > 0) { APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf)); }
    else { APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf)); }
}

static void APP_RTU_AT_CEREG(void)
{
    char buf[16] = "AT+CEREG?\r\n";
    APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
}

static void APP_RTU_AT_MIPOPEN(void)
{
    char buf[RTU_TX_AT_LEN] = {0};
    int ret = -1;
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_rtu_at.tcp_cnt[i] == 0)
        {
            g_app_rtu_at.tcp_cnt_chl = i;
            sprintf(buf, "AT+MIPOPEN=%d,\"TCP\",%s,%d\r\n", i, g_app_dtu_ip[i].ip, g_app_dtu_ip[i].port);
            APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
            LOGT("Connecting to IP[%d]:\"%s\",port:%d\n", i, g_app_dtu_ip[i].ip, g_app_dtu_ip[i].port);
            ret = 0;
            break;
        }
    }
    if (ret < 0)
    {
        LOGT("err: tcp open\n");
        APP_RTU_AT_Rec_Cfg_Next();
    }
}

static void APP_RTU_AT_MIPCLOSE(void)
{
    char buf[48] = {0};
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_rtu_at.tcp_close[i] > 0)
        {
            g_app_rtu_at.tcp_close[i] = 0;
            sprintf(buf, "AT+MIPCLOSE=%d\r\n", i);
            APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
            break;
        }
    }
}

int APP_RTU_AT_Ready_Chk(void)
{
    int ret = -1;
    g_app_rtu_at.poweron_chk++;
    if (g_app_rtu_at.poweron == 0)
    {
        if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF)
        {
            if (g_app_rtu_at.poweron_chk % 1200 == 0)
            {
                g_app_rtu_at.poweron_chk_th = 40;
                APP_RTU_AT_RESET(0);
                LOGT("reboot:rtu\n");
            }
            else if (g_app_rtu_at.poweron_chk % g_app_rtu_at.poweron_chk_th == 0)
            {
                g_app_rtu_at.poweron_chk_th = 15;
                APP_RTU_AT();
            }
        }
    }
    else if (g_app_rtu_at.poweron == 2)
    {
        if (g_app_rtu_at.poweron_chk % 100 == 0)
        {
            if (g_app_dtu_ip[g_app_rtu_at.tcp_cnt_chk].en > 0)
            {
                APP_RTU_AT_MIPSTATE(g_app_rtu_at.tcp_cnt_chk, 0);
            }
            else if (strlen(g_app_rtu_sim.iccid) < 10)
            {
                APP_RTU_AT_ICCID();
            }

            if (++g_app_rtu_at.tcp_cnt_chk > 3)
            {
                g_app_rtu_at.tcp_cnt_chk = 0;
                g_app_rtu_at.poweron_chk = 230;
                APP_RTU_AT_CSQ();
            }
            else
            {
                g_app_rtu_at.poweron_chk -= 10;
            }
        }
    }
    else
    {
        ret = 0;
    }

    if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF && g_app_rtu_at.poweron > 0)
    {
        if (++g_app_rtu_at.net_timeout_cnt > 300)
        {
            g_app_rtu_at.net_timeout_cnt = 0;
            APP_RTU_AT_ENCOD_Chl(0);
        }
    }
    else
    {
        g_app_rtu_at.net_timeout_cnt = 0;
    }

    return ret;
}

void APP_RTU_AT_Config_Handle(void)
{
    if (APP_RTU_AT_Ready_Chk() == 0)
    {
        switch (g_app_rtu_at.step_cfg)
        {
        case RTU_AT_STEP4:  APP_RTU_AT_MIPOPEN();  break;
        case RTU_AT_STEP6:  APP_RTU_AT_ICCID();   break;
        case RTU_AT_STEP7:  APP_RTU_AT_CSQ();     break;
        case RTU_AT_STEP9:  APP_RTU_AT_MIPCLOSE();break;
        case RTU_AT_STEP10: APP_RTU_AT_CEREG();   break;
        case RTU_AT_STEP11: APP_RTU_AT_ENCOD();   break;
        default: break;
        }

        if (g_app_rtu_at.step_cfg < RTU_AT_STEP_MAX)
        {
            g_app_rtu_at.step_cfg = RTU_AT_STEP_MAX;
        }
    }
}

void APP_RTU_AT_Config_Handle_Err(void)
{
    g_app_rtu_at.poweron = 2;
}

uint8_t  APP_RTU_AT_Chk_Ready(void)
{
    return g_app_rtu_at.poweron;
}

void APP_RTU_Tcp_Cnt_Disconnect(uint8_t chl)
{
    g_app_rtu_at.tcp_cnt[chl] = 0;
}

void APP_RTU_AT_Init(void)
{
    BSP_TIMER_Init(&g_rtu_at_timer, APP_RTU_AT_timer, TIMEOUT_2S, 0);
    BSP_TIMER_Init(&g_rtu_at_config_timer, APP_RTU_AT_Config_Handle, 100, 100);
    BSP_TIMER_Start(&g_rtu_at_config_timer);
    BSP_TIMER_Init(&g_rtu_at_timer_err, APP_RTU_AT_Config_Handle_Err, TIMEOUT_10S, 0);
}
