#include "app_rtu_at_cmd.h"
#include "app_rtu_at_core.h"   /* 引入核心流程函数原型 */
#include "app_config.h"
#include "app_dtu_reconnect.h" // 新增：URC断开通知重连模块

app_rtu_sim_def g_app_rtu_sim = {0};

static const app_dtu_ip_def g_app_dtu_ip_default =
{
    .md = 0, .en = 1, .ip = "colnew.dltengyi.cn", .port = 9999,
};

app_dtu_ip_def g_app_dtu_ip[RTU_AT_CH_SUM] =
{
    { .en = 1, .md = 0, .ip = "colnew.dltengyi.cn", .port = 9999, },
    { .md = 0, .ip = "1", .port = 1, },
    { .md = 0, .ip = "1", .port = 1, },
    { .md = 0, .ip = "1", .port = 1, },
};

static app_rtu_tx_def g_app_rtu_tx = {0};
static uint8_t g_atBuf[512];

/* 多通道发送 */
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size)
{
    if (g_app_rtu_at.poweron > 1 && chl < RTU_AT_CH_SUM)
    {
        memset(g_app_rtu_tx.txBuf, 0, RTU_TX_LEN);
        sprintf(g_app_rtu_tx.txBuf, RTU_AT_CMD_SEND, chl);
        g_app_rtu_tx.txNum = strlen(g_app_rtu_tx.txBuf);
        for (g_app_rtu_tx.index = 0; g_app_rtu_tx.index < size; g_app_rtu_tx.index++)
        {
            sprintf(g_app_rtu_tx.txBuf + g_app_rtu_tx.txNum + g_app_rtu_tx.index * 2, "%02X", buffer[g_app_rtu_tx.index]);
        }
        g_app_rtu_tx.txNum += size * 2;
        g_app_rtu_tx.txBuf[g_app_rtu_tx.txNum++] = '\r';
        g_app_rtu_tx.txBuf[g_app_rtu_tx.txNum++] = '\n';

        APP_RTU_AT_Tx((uint8_t *)g_app_rtu_tx.txBuf, g_app_rtu_tx.txNum);
    }
    else if (g_app_rtu_at.poweron == 1)
    {
        return;
    }
    else
    {
        APP_RTU_AT_Tx(buffer, size);
    }

    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOG("g_app_rtu_at.poweron[%d],chl[%d]\n", g_app_rtu_at.poweron, chl);
        if (g_app_rtu_at.poweron > 1) { LOG_HEX(g_app_rtu_tx.txBuf, g_app_rtu_tx.txNum); }
        else { LOG_HEX(buffer, size); }
    }
}

/* 识别应答类别 */
static int APP_RTU_AT_Rec_Chk(uint8_t *buffer)
{
    int ret = -1;
    char *p = strchr((char *)buffer, '+');
    static const char *g_rtu_cmd_buf[] = {
        RTU_AT_CMD_URC, RTU_AT_MIPSEND, RTU_AT_MATREADY, RTU_AT_MIPSTATE,
        RTU_AT_MIPOPEN, RTU_AT_MIPMODE, RTU_AT_MCCID, RTU_AT_CSQ,
        RTU_AT_MREBOOT, RTU_AT_MIPCLOSE, RTU_AT_CEREG, "NULL"
    };
    if (p != NULL)
    {
        for (int i = 0; i < 12; i++)
        {
            if (strstr(p, g_rtu_cmd_buf[i]) != NULL)
            {
                ret = i;
                break;
            }
        }
    }
    return ret;
}

static int APP_RTU_AT_Rec_Chk_Ret(uint8_t *buffer)
{
    int ret = -1;
    char *p = (char *)buffer;
    if (strstr((char *)p, RTU_AT_OK) != NULL) { ret = 0; }
    else
    {
        p = strstr((char *)p, RTU_AT_ERR);
        if (p != NULL)
        {
            p += strlen(RTU_AT_ERR);
            ret = atoi(p);
            if (g_app_rtu_at.poweron == 2) { LOGT("err:rtu code[%d]\n", ret); }
        }
    }
    return ret;
}

/* MIPOPEN 解析 */
static int APP_RTU_AT_Rec_Chk_MIPOPEN(uint8_t *buffer)
{
    int ret = -1;
    int chl = 0;
    char *p = strstr((char *)buffer, RTU_AT_MIPOPEN);
    if (p != NULL)
    {
        p = p + strlen(RTU_AT_MIPOPEN) +2;
        chl = atoi(p);
        if (chl < 0 || chl >= RTU_AT_CH_SUM)
        {
            LOG("ERROR: MIPOPEN invalid channel: %d (max: %d)\r\n", chl, RTU_AT_CH_SUM - 1);
            return -1;
        }
        p = strchr(p, ',') + 1;
        ret = atoi(p);
        if (ret == 0)
        {
            LOGT("IP[%d] connected:\"%s\",%d\n", chl, g_app_dtu_ip[chl].ip, g_app_dtu_ip[chl].port);
        }
        else
        {
            LOGT("IP[%d] connection failed.\n", chl);
        }
    }
    if (ret == 0 && chl >= 0 && chl < RTU_AT_CH_SUM) { g_app_rtu_at.tcp_cnt[chl] = 1; }
    return ret;
}

/* MIPCLOSE 解析 */
static int APP_RTU_AT_Rec_Chk_MIPCLOSE(uint8_t *buffer)
{
    int ret = -1;
    int chl = -1;
    char *p = strstr((char *)buffer, RTU_AT_MIPCLOSE);
    if (p != NULL)
    {
        p = p + strlen(RTU_AT_MIPCLOSE) +2;
        chl = atoi(p);
        LOGT("IP[%d] disconnected\n", chl);
        ret = 0;
        APP_RTU_Tcp_Cnt_Disconnect((uint8_t)chl);
        if (chl >= 0) { DTU_Reconnect_OnTcpDisconnect((uint8_t)chl); } // 新增：通知重连模块
    }
    return ret;
}

/* MIPSTATE 解析并触发重连 */
static int APP_RTU_AT_Rec_Chk_MIPSTATE(uint8_t *buffer)
{
    int ret = -1;
    char *p = strstr((char *)buffer, RTU_AT_MIPSTATE) + strlen(RTU_AT_MIPSTATE) +2;
    if (p != NULL)
    {
        ret = atoi(p);
        p = strrchr(p, ',') + 1;
        if (strstr(p, "CONNECTED") == 0)
        {
            APP_RTU_AT_MIPOPEN_Chl(ret);
            DTU_Reconnect_OnTcpDisconnect((uint8_t)ret); // 新增：状态非CONNECTED也触发断链通知
            char *p1 = strrchr(p, '\"');
            if (p1 != NULL)
            {
                char err_buf[11] = {0};
                memcpy(err_buf, p + 1, p1 - p - 1);
                LOGT("IP[%d]:%s\n", ret, err_buf);
            }
        }
    }
    return ret;
}

/* ICCID 解析 */
static int APP_RTU_AT_Rec_Chk_ICCID(uint8_t *buffer)
{
    int ret = -1;
    char *p1 = strstr((char *)buffer, RTU_AT_MCCID) +2 + strlen(RTU_AT_MCCID);
    if (p1 != NULL && strlen(p1) > 20)
    {
        memcpy(g_app_rtu_sim.iccid, p1, 20);
        ret = 0;
        LOGT("iccid:%s\n", g_app_rtu_sim.iccid);
    }
    else
    {
        LOGT("iccid err:%s\n", buffer);
    }
    return ret;
}

/* CSQ 解析 */
static int APP_RTU_AT_Rec_Chk_CSQ(uint8_t *buffer)
{
    int ret = -1;
    char *p1 = strstr((char *)buffer, RTU_AT_CSQ) + strlen(RTU_AT_CSQ) +2;
    if (p1 != NULL && strlen(p1) > 1)
    {
        g_app_rtu_sim.signal = atoi(p1);
        if (g_app_rtu_sim.signal > 32) { g_app_rtu_sim.signal_per = 0; }
        else { g_app_rtu_sim.signal_per = g_app_rtu_sim.signal * 100 / (30); }
        ret = 0;
        if (g_app_rtu_sim.signal_per != g_app_rtu_sim.signal_per_last)
        {
            g_app_rtu_sim.signal_per_last = g_app_rtu_sim.signal_per;
            LOGT("signal:%d-%d\n", g_app_rtu_sim.signal, g_app_rtu_sim.signal_per);
            LOG("\n");
        }
    }
    return ret;
}

/* MREBOOT 解析 */
static uint8_t APP_RTU_AT_Rec_Chk_MREBOOT(uint8_t *buffer)
{
    uint8_t ret = 0;
    char *p1 = (char *)buffer;
    if (p1 != NULL)
    {
        LOGT("rtu reboot..\n");
        ret = 1;
    }
    return ret;
}

/* CEREG 解析（新增实现，解决链接错误） */
int APP_RTU_AT_Rec_Chk_CEREG(uint8_t *buffer)
{
    int ret = -1;
    char *p1 = strchr((char *)buffer, ',') + 1;
    if (p1 != NULL)
    {
        ret = atoi(p1);
        if (ret == 1 || ret == 5)
        {
            ret = 1;
        }
    }
    return ret;
}

/* AT接收入口 */
void APP_RTU_AT_Rec(uint8_t *buffer, uint16_t len)
{
    int step = APP_RTU_AT_Rec_Chk(buffer);
    int ret = -1;
    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOG("rtu r:%s\n", buffer);
        LOGT("rtu r step[%d]\n", step);
    }

    if (step < 0)
    {
        ret = APP_RTU_AT_Rec_Chk_Ret(buffer);
        if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP9)
        {
            step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
        }
        else if (ret == 0 && g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP11)
        {
            step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
        }
        else if (ret == 552)
        {
            if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP4)
            {
                step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
                g_app_rtu_at.tcp_cnt[g_app_rtu_at.tcp_cnt_chl] = 1;
            }
            else if (step == RTU_AT_STEP4)
            {
                g_app_rtu_at.tcp_cnt[g_app_rtu_at.tcp_cnt_chl] = 1;
            }

            LOGT("err:cnt 552\n");
        }
        else
        {
            return;
        }
    }

    switch (step)
    {
    case RTU_AT_STEP0:
        break;
    case RTU_AT_STEP1:
        break;
    case RTU_AT_STEP2:
        LOGT("Network Module\n");
        APP_RTU_AT_Poweron();
        break;
    case RTU_AT_STEP3:
        APP_RTU_AT_Rec_Chk_MIPSTATE(buffer);
        break;
    case RTU_AT_STEP4:
        APP_RTU_AT_Rec_Chk_MIPOPEN(buffer);
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    case RTU_AT_STEP6:
        APP_RTU_AT_Rec_Chk_ICCID(buffer);
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    case RTU_AT_STEP7:
        APP_RTU_AT_Rec_Chk_CSQ(buffer);
        g_app_rtu_at.step_cfg = RTU_AT_STEP_SUCCESS;
        g_app_rtu_at.poweron = 2;
        break;
    case RTU_AT_STEP8:
        APP_RTU_AT_Rec_Chk_MREBOOT(buffer);
        break;
    case RTU_AT_STEP9:
        APP_RTU_AT_Rec_Chk_MIPCLOSE(buffer);     // 新增：内部会通知重连模块
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    case RTU_AT_STEP10:
        if (APP_RTU_AT_Rec_Chk_CEREG(buffer) > 0)
        {
            APP_RTU_AT_Rec_Cfg_Next();
        }
        else
        {
            g_app_rtu_at.tx_repeat = 3;
        }
        break;
    case RTU_AT_STEP11:
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    }
}

/* IP/CSQ/ICCID 等接口保持不变 */

void APP_RTU_AT_Ip_Set(uint8_t ch, char* ip, uint16_t pt, uint8_t sta)
{
    if (ch > 3)
    {
        LOGT("err:ch 0~3\n");
        return;
    }
    uint8_t len = strlen(ip);
    if (sta == 1 || sta == 2) { memset(&g_app_dtu_ip[ch], 0, sizeof(app_dtu_ip_def)); }
    if (pt > 0 && len > 1)
    {
        g_app_dtu_ip[ch].en = 1;
        g_app_dtu_ip[ch].port = pt;
        memcpy(g_app_dtu_ip[ch].ip, ip, len);
    }
    else
    {
        g_app_dtu_ip[ch].en = 0;
        g_app_dtu_ip[ch].port = 0;
    }
    if (sta == 3 || sta == 2) { APP_RTU_AT_MIPCLOSE_Chl(ch, 1); }
    LOGT("new ip ch:%d,\"%s\",%d\n", ch, g_app_dtu_ip[ch].ip, g_app_dtu_ip[ch].port);
}

app_dtu_ip_def APP_RTU_AT_Ip_Get_Chl(uint8_t ch)
{
    if (ch < 4)
    {
        LOGT("read ip ch:%d,\"%s\",%d\n", ch, g_app_dtu_ip[ch].ip, g_app_dtu_ip[ch].port);
        return g_app_dtu_ip[ch];
    }
    else
    {
        LOGT("err:ch 0~3\n");
        app_dtu_ip_def ip = {0};
        return ip;
    }
}

void APP_RTU_AT_Ip_Get_All(void)
{
    for (int i = 0; i < RTU_AT_CH_SUM; i++) { /* 可接入持久化读取 */ }
}

void APP_RTU_AT_Ip_Default(void)
{
    memset(g_app_dtu_ip, 0, 4 * sizeof(app_dtu_ip_def));
    for (int i = 0; i < 4; i++)
    {
        if (i == 0) { memcpy(&g_app_dtu_ip[i], &g_app_dtu_ip_default, sizeof(app_dtu_ip_def)); }
        else { g_app_dtu_ip[i].en = 0; g_app_dtu_ip[i].ip[0] = 119; }
    }
    LOGT("ip default!\n");
}

void APP_RTU_AT_MIPCLOSE_Chl(uint8_t chl, uint8_t rpt)
{
    char buf[48] = {0};
    sprintf(buf, "AT+MIPCLOSE=%d\r\n", chl);
    if (rpt > 0) { APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf)); }
    else { APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf)); }
}

void APP_RTU_AT_CSQ(void)
{
    char buf[32] = "AT+CSQ\r\n";
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
}

void APP_RTU_AT_ICCID(void)
{
    char buf[32] = "AT+MCCID\r\n";
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
}

/* 编码修改：通过 core 导出的 APP_RTU_AT_ENCOD */
void APP_RTU_AT_ENCOD_Chl(int chl)
{
    g_app_rtu_at.tcp_code[chl] = 0;
    APP_RTU_AT_ENCOD();
}

int APP_RTU_AT_Rec_Chk_MIPCLOSE_To(uint8_t *buffer)
{
    int ret = -1;
    uint8_t chl = 0, err = 0;
    char *p = strstr((char *)buffer, RTU_AT_CMD_URC_TCP_DIS);
    if (p != NULL)
    {
        p += strlen(RTU_AT_CMD_URC_TCP_DIS) +1;
        chl = atoi(p);
        p += 2;
        err  = atoi(p);
        LOGT("tcp remote discnt, chl[%d],sta[%d]\n", chl, err);
        if (chl < 4) { APP_RTU_Tcp_Cnt_Disconnect(chl); DTU_Reconnect_OnTcpDisconnect(chl); } // 新增通知
        ret = 0;
    }
    return ret;
}

app_rtu_rx_def APP_RTU_AT_Rx_Chl(uint8_t *buffer, uint16_t size)
{
    app_rtu_rx_def rx = {0};
    if (g_app_rtu_at.poweron > 1)
    {
        char *p1 = strstr((char *)buffer, RTU_AT_CMD_URC);
        if (p1 != NULL)
        {
            if (strstr(p1, RTU_AT_CMD_URC_TCP) != NULL)
            {
                p1 = strstr(p1, RTU_AT_CMD_URC_TCP) + strlen(RTU_AT_CMD_URC_TCP) +1;
                rx.chl = atoi(p1);
                p1 += 2;
                rx.rxLen = atoi(p1);
                p1 = strchr(p1, ',') + 1;
                if (rx.rxLen < RTU_RX_LEN) { memcpy(rx.rxBuf, p1, rx.rxLen); }
                else { LOGT("err: rtu rx len > [%d]\n", RTU_RX_LEN); }
            }
            else if (strstr(p1, RTU_AT_CMD_URC_TCP_DIS) != NULL)
            {
                APP_RTU_AT_Rec_Chk_MIPCLOSE_To(buffer);
            }
            else
            {
                LOGT("err:rtu rx: %s\n", (char *)buffer);
            }
        }
        else
        {
            memset(g_atBuf,0,512);
            memcpy(g_atBuf,buffer,size);
            APP_RTU_AT_Rec(g_atBuf, size);
        }
    }
    else
    {
        if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF)
        {
            memset(g_atBuf,0,256);
            memcpy(g_atBuf,buffer,size);
            APP_RTU_AT_Rec(g_atBuf, size);
        }
        rx.rxLen = size;
        memcpy(rx.rxBuf, buffer, rx.rxLen);
    }
    return rx;
}