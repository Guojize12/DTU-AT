#include "app_config.h"
#include "app_rtu_at.h"

//#define USE_RTU_CACHE //none

// Forward declaration for hard power reboot function
void BSP_DTU_Power_Reboot(void);

// 4G模块故障恢复机制常量定义
#define AT_FAIL_LIMIT 10                   // AT连续失败次数上限，超过则软重启
#define SOFT_RESET_LIMIT 3                 // 软重启次数上限，超过则硬重启
#define HARD_RESET_MIN_GAP_TICKS 1200      // 硬重启最小间隔(100ms ticks) = 2分钟

// 计算自上次硬重启以来的tick数，处理首次重启和计数器回绕
static inline uint32_t get_ticks_since_last_reboot_at(uint32_t current_tick, uint32_t last_reboot_tick)
{
    if (last_reboot_tick == 0) {
        return HARD_RESET_MIN_GAP_TICKS; // 首次重启，返回满足条件的值
    }
    return (current_tick - last_reboot_tick); // 无符号减法自动处理回绕
}

#define  RTU_AT_CMD_SEND          "AT+MIPSEND=%d,0,"  //通道 发 

#define  RTU_AT_CMD_URC           "+MIPURC"         //接收数据头
#define  RTU_AT_CMD_URC_TCP       "\"rtcp\""        //接收TCP数据提示
#define  RTU_AT_CMD_URC_TCP_DIS   "\"disconn\""         //TCP连接异常提示

#define  RTU_AT_OK            "OK"        //成功 
#define  RTU_AT_ERR           "ERROR:"     //错误 

//开机打印信息
#define  RTU_AT_MATREADY      "+MATREADY"
// [2025-01-13 14:12:56.584]# SEND ASCII(4)>
// AT
// [2025-01-13 14:12:56.616]# RECV ASCII 19B>
// +MATREADY
// OK

//TCP/IP参数设置
#define  RTU_AT_MIPCFG       "+MIPCFG"
// AT+MIPCFG=?

// +MIPCFG: "cid",(0-5),(1-15)
// +MIPCFG: "encoding",(0-5),(0-2),(0-1)
// +MIPCFG: "timeout",(0-5),(1-120)
// +MIPCFG: "rcvbuf",(0-5),(1460-65536)
// +MIPCFG: "autofree",(0-5),(0-1)
// +MIPCFG: "ackmode",(0-5),(0-1)
// +MIPCFG: "ssl",(0-5),(0-1),(0-5)
// OK

//查询TCP/IP连接状态
#define  RTU_AT_MIPSTATE      "+MIPSTATE"
// AT+MIPSTATE=0

// +MIPSTATE: 0,"TCP","120.27.12.119",2040,"CONNECTED"
// +MIPSTATE: 1,"TCP","120.27.12.119",2040,"CONNECTING"
// +MIPSTATE: 2,,,,"INITIAL"
// +MIPSTATE: 3,"UDP","120.27.12.119",2040,"CONNECTED"
// +MIPSTATE: 4,,,,"INITIAL"
// OK

// "INITIAL",//未建立连接
// "CONNECTING",//正在建立连接
// "CONNECTED",//已建立连接
// "CLOSING",//正在关闭连接
// "CLOSED",//连接已关闭


//建立TCP/IP连接
#define  RTU_AT_MIPOPEN       "+MIPOPEN"
// AT+MIPOPEN=0,"TCP","112.125.89.8",47696

// [2025-01-13 14:11:55.438]# RECV ASCII 6B>
// OK
// [2025-01-13 14:11:55.674]# RECV ASCII 17B>
// +MIPOPEN: 0,0

//切换数据模式
#define  RTU_AT_MIPMODE        "+MIPMODE"
// AT+MIPMODE=0,0

// OK

//关闭TCP/IP连接
#define  RTU_AT_MIPCLOSE        "+MIPCLOSE"
//  AT+MIPCLOSE=0
//  OK
//  +MIPCLOSE: 0

//发送数据
#define  RTU_AT_MIPSEND       "+MIPSEND"
// AT+MIPSEND=0,10,"0123456789"

// +MIPSEND: 0,10
// OK

//接收数据
#define  RTU_AT_MIPURC        "+MIPURC"
// +MIPURC: "rtcp",0,5,qweqe


//重启
#define  RTU_AT_MREBOOT       "+MREBOOT"
// AT+MREBOOT

// OK
// REBOOTING

//查询设备ICCID号
#define  RTU_AT_MCCID       "+MCCID"
// AT+ICCID?

// +MCCID: 89860818102381421186
// OK

//查询设备网路信号强度
#define  RTU_AT_CSQ         "+CSQ"
// AT+CSQ
// +CSQ: 13,99
// OK

// 0        -113 dBm or less
// 1        -111 dBm
// 2..30    -109. . . -53 dBm
// 31       -51 dBm or greater
// 99       未知或无法检测

//查询网络注册状态
#define  RTU_AT_CEREG       "+CEREG"
// AT+CEREG?
// +CEREG: 0,1
// OK

// 0：未注册，但设备现在并不在搜寻新的运营商网络
// 1：已注册本地网络
// 2：未注册，设备正在搜寻基站
// 4：未知状态
// 5：已注册，但是处于漫游状态

#define  RTU_AT_DET_NUM     380  //rtu检测超时

const uint16_t g_rtu_err_code[22] =
{
    550, //TCP/IP 未知错误
    551, //TCP/IP 未被使用
    552, //TCP/IP 已被使用
    553, //TCP/IP 未连接
    554, //SOCKET 创建失败
    555, //SOCKET 绑定失败
    556, //SOCKET 监听失败
    557, //SOCKET 连接被拒绝
    558, //SOCKET 连接超时
    559, //SOCKET 连接失败（其他异常）
    560, //SOCKET 写入异常
    561, //SOCKET 读取异常
    562, //SOCKET 接受异常
    570, //PDP 未激活
    571, //PDP 激活失败
    572, //PDP 去激活失败
    575, //APN 未配置
    576, //端口忙碌
    577, //不支持的IPV4/IPV6
    580, //DNS解析失败或错误的IP格式
    581, //DNS忙碌
    582, //PING忙碌
};

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

static const char g_rtu_cmd_buf[RTU_AT_STEP_MAX][24] =
{
    RTU_AT_MIPURC,    //接收数据 0
    RTU_AT_MIPSEND,   //发送数据 1
    RTU_AT_MATREADY,  //开机打印信息 2

    RTU_AT_MIPSTATE,  //查询TCP/IP连接状态 3
    RTU_AT_MIPOPEN,   //建立TCP/IP连接 4
    RTU_AT_MIPMODE,   //切换数据模式 5
    RTU_AT_MCCID,     //查询设备ICCID号 6
    RTU_AT_CSQ,       //查询设备网路信号强度 7
    RTU_AT_MREBOOT,   //重启 8
    RTU_AT_MIPCLOSE,  //关闭TCP/IP连接 9
    RTU_AT_CEREG,     //查询网络注册状态 10
    "NULL",
};

app_rtu_sim_def g_app_rtu_sim = {0};

//默认配置
static const app_dtu_ip_def g_app_dtu_ip_default =
{
    .md = 0,
    .en = 1,
    .ip = "colnew.dltengyi.cn",
    .port = 9999,
};

app_dtu_ip_def g_app_dtu_ip[RTU_AT_CH_SUM] =
{
    //通道A
    {
        .en = 1,
        .md = 0,
        .ip = "colnew.dltengyi.cn",
        .port = 9999,
    },
    //通道B
    {
        .md = 0,
        .ip = "1",
        .port = 1,
    },
    //通道C
    {
        .md = 0,
        .ip = "1",
        .port = 1,
    },
    //通道D
    {
        .md = 0,
        .ip = "1",
        .port = 1,
    },
};

app_rtu_at_def g_app_rtu_at =
{
    .poweron_chk_th = 35,

};

static Timer g_rtu_at_timer = {0};
static Timer g_rtu_at_config_timer = {0};
static Timer g_rtu_at_timer_err = {0};



/*适配接口 **/

//发送
void APP_RTU_AT_Tx(uint8_t *buffer, uint16_t size)
{
    APP_DTU_Send_Buf(buffer, size); //ttl
}

//与平台连接状态 0断开(默认),1连接
int APP_RTU_AT_Chk_Cntn_Sta(void)
{
    return g_dtu_cmd.net_status;
}
//通道IP设置  ch:0~3   sta,0,1复制,2复制立即生效,3立即生效
void APP_RTU_AT_Ip_Set(uint8_t ch, char* ip, uint16_t pt, uint8_t sta)
{
    if (ch > 3)
    {
        LOGT("err:ch 0~3\n");
        return;
    }
    uint8_t len = strlen(ip);
    if (sta == 1 || sta == 2)
    {
        memset(&g_app_dtu_ip[ch], 0, sizeof(app_dtu_ip_def));
    }
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

    if (sta == 3 || sta == 2)
    {
        APP_RTU_AT_MIPCLOSE_Chl(ch, 1); //立即生效
    }
    LOGT("new ip ch:%d,\"%s\",%d\n", ch, g_app_dtu_ip[ch].ip, g_app_dtu_ip[ch].port);
//  TODO  APP_CONFIG_Ip_Write(ch,g_app_dtu_ip[ch]);//单ip写入
}
//通道IP读取
app_dtu_ip_def APP_RTU_AT_Ip_Get_Chl(uint8_t ch)
{
    if (ch < 4)
    {
        //  TODO APP_CONFIG_Ip_Read(ch,&g_app_dtu_ip[ch]);//ip读取
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
//通道IP读取
void APP_RTU_AT_Ip_Get_All(void)
{
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        //  TODO APP_CONFIG_Ip_Read(i,&g_app_dtu_ip[i]);//ip读取
    }
}

//默认IP
void APP_RTU_AT_Ip_Default(void)
{
    memset(g_app_dtu_ip, 0, 4 * sizeof(app_dtu_ip_def));
    for (int i = 0; i < 4; i++)
    {
        if (i == 0)
        {
            memcpy(&g_app_dtu_ip[i], &g_app_dtu_ip_default, sizeof(app_dtu_ip_def));
        }
        else
        {
            g_app_dtu_ip[i].en = 0;
            g_app_dtu_ip[i].ip[0] = 119;
        }
        //  TODO APP_CONFIG_Ip_Write(i,g_app_dtu_ip[i]);//ip写入
    }
    LOGT("ip default!\n");
}

/*适配接口 **/

#ifdef USE_RTU_CACHE
///缓存发送
#define RTU_RING_LEN  10  //缓存深度

static Timer g_timer_rtu_cache = {0};
static uint8_t g_rtu_ringbuf[RTU_RING_LEN][256]; //缓存大小
static uint8_t g_rtu_buff_index;//缓存序号
static uint8_t g_rtu_rb_buff[RTU_RING_LEN + 1]; //环形缓冲区
static RingBuf_t g_rtu_rb_t = {0};

static void APP_RTU_AT_Tx_Chl_Cache(uint8_t chl, uint8_t *buffer, uint16_t size)
{
    if (g_app_rtu_at.at_status > 0) //at模式
    {
        LOGT("err:rtu in at mode\n");
        return;
    }

    if (g_app_rtu_at.poweron > 1 && g_app_rtu_at.mode == 0) //rtu有效,使用多通道发
    {
        uint8_t tx_buf[256] = {0};
        uint16_t tx_len = 0;

        sprintf((char*)tx_buf, RTU_AT_CMD_SEND, chl, size);
        tx_len = strlen((char*)tx_buf);
        memcpy(tx_buf + tx_len, buffer, size);
        tx_len += size;
        APP_RTU_AT_Tx(tx_buf, tx_len);
    }
    else //兼容其它直接透传
    {
        APP_RTU_AT_Tx(buffer, size);
    }
}

static void APP_RTU_Cache_Send_Callback(void)
{
    int8_t index = ringbuf_read(&g_rtu_rb_t);
    if (index >= 0) //有缓存
    {
        APP_RTU_AT_Tx_Chl_Cache(g_rtu_ringbuf[index][254], g_rtu_ringbuf[index], g_rtu_ringbuf[index][255]);
//          LOG_HEX(g_dtu_ringbuf[index],g_dtu_ringbuf[index][255]);
    }
}

//多通道发送
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size)
{
    if (g_rtu_buff_index >= RTU_RING_LEN)
    {
        g_rtu_buff_index = 0;
    }
    ringbuf_write(&g_rtu_rb_t, g_rtu_buff_index); //记录缓存序号
    memcpy(g_rtu_ringbuf[g_rtu_buff_index], buffer, size); //缓存数据
    g_rtu_ringbuf[g_rtu_buff_index][255] = size; //缓存大小
    g_rtu_ringbuf[g_rtu_buff_index][254] = chl; //发送通道号
    g_rtu_buff_index++;
    LOG("g_rtu_buff_index[%d]\n", g_rtu_buff_index);
}

#else

static app_rtu_tx_def g_app_rtu_tx = {0};

//AT+MIPSEND=0,10,"1234567890"
//多通道数据发送 通道0~3
void APP_RTU_AT_Tx_Chl(uint8_t chl, uint8_t *buffer, uint16_t size)
{

    if (g_app_rtu_at.poweron > 1 && chl < RTU_AT_CH_SUM) //rtu有效,使用多通道发
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
    else if (g_app_rtu_at.poweron == 1) //等待AT配置
    {
        return;
    }
    else //兼容其它直接透传
    {
        APP_RTU_AT_Tx(buffer, size);
//          LOG_HEX(buffer,size);
    }

    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOG("g_app_rtu_at.poweron[%d],chl[%d]\n", g_app_rtu_at.poweron, chl);
        if (g_app_rtu_at.poweron > 1)
        {
            LOG_HEX(g_app_rtu_tx.txBuf, g_app_rtu_tx.txNum);
        }
        else
        {
            LOG_HEX(buffer, size);
        }
    }

}
#endif

//+MIPURC: "disconn",0,1
//1 服务器关闭连接。
//2 连接异常
//3 PDP去激活
//被动关闭TCP/IP连接   RTU_AT_MIPCLOSE
int APP_RTU_AT_Rec_Chk_MIPCLOSE_To(uint8_t *buffer)
{
    int ret = -1;
    uint8_t chl = 0, err = 0;
    char *p = strstr((char *)buffer, RTU_AT_CMD_URC_TCP_DIS);
    if (p != NULL) //指令结果
    {
        p += strlen(RTU_AT_CMD_URC_TCP_DIS) +1;
        chl = atoi(p);
        p += 2;
        err  = atoi(p);
        LOGT("tcp remote discnt, chl[%d],sta[%d]\n", chl, err);
        if (chl < 4)
        {
            APP_RTU_Tcp_Cnt_Disconnect(chl); //断开连接记录
        }

        ret = 0; //退出
    }
    return ret;
}
static uint8_t g_atBuf[512];
//+MIPURC: "rtcp",0,4,sfdf
//多通道收 解析
//接收长度>0时,为远程数据
app_rtu_rx_def APP_RTU_AT_Rx_Chl(uint8_t *buffer, uint16_t size)
{
    app_rtu_rx_def rx = {0};
//      LOG("buffer:%s\n",buffer);
    if (g_app_rtu_at.poweron > 1) //rtu有效
    {
        char *p1 = strstr((char *)buffer, RTU_AT_CMD_URC);

        if (p1 != NULL)
        {
            if (strstr(p1, RTU_AT_CMD_URC_TCP) != NULL)
            {
                p1 = strstr(p1, RTU_AT_CMD_URC_TCP) + strlen(RTU_AT_CMD_URC_TCP) +1;
                rx.chl = atoi(p1); //接收通道
                p1 += 2;
                rx.rxLen = atoi(p1); //接收长度
                p1 = strchr(p1, ',') + 1;
                if (rx.rxLen < RTU_RX_LEN)
                {
                    memcpy(rx.rxBuf, p1, rx.rxLen); //接收数据
                }
                else
                {
                    LOGT("err: rtu rx len > [%d]\n", RTU_RX_LEN);
                }
#if 0
                LOGT("rtu r:ch[%d],len[%d]\n", rx.chl, rx.rxLen);
                LOG_HEX(buffer, size);
                LOG_HEX(rx.rxBuf, rx.rxLen);
#endif
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
        if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF) //网络未连接
        {
            memset(g_atBuf,0,256);
            memcpy(g_atBuf,buffer,size);
            APP_RTU_AT_Rec(g_atBuf, size);
        }
        rx.rxLen = size;
        memcpy(rx.rxBuf, buffer, rx.rxLen); //接收数据
    }
    return rx;
}

void APP_RTU_AT_Rec_Cfg_Next(void)
{
    if (g_app_rtu_at.poweron < 2 || g_app_rtu_at.step_next < RTU_TX_LIST_LEN - 1)
    {
        g_app_rtu_at.step_cfg = g_app_rtu_at.cmd_list[++g_app_rtu_at.step_next];
    }
}

//rtu无应答超时
static void APP_RTU_AT_timer(void)
{
    if (g_app_rtu_at.tx_repeat > 0) //重发
    {
#if 0
        LOGT("at repeat[%d]:%s\n", g_app_rtu_at.tx_repeat, g_app_rtu_at.tx_buf);
#endif
        if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP4 //连接
                || g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP9) //关闭
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
        //超时 退出at
        g_app_rtu_at.step_cfg = g_app_rtu_at.cmd_list[++g_app_rtu_at.step_next]; //下一步
        if (g_app_rtu_at.step_cfg == 0) //完成
        {
            g_app_rtu_at.step_cfg = RTU_AT_STEP_FAIL;
        }
    }
}

//AT指令超时发送
static void APP_RTU_AT_Tx_To(uint8_t *buffer, uint16_t size)
{
    //超时重发 开启
    BSP_TIMER_Stop(&g_rtu_at_timer);
    BSP_TIMER_Start(&g_rtu_at_timer);
    g_app_rtu_at.tx_repeat = 3;

    //发送备份
    g_app_rtu_at.tx_len = size;
    memset(g_app_rtu_at.tx_buf, 0, RTU_TX_AT_LEN);
    memcpy(g_app_rtu_at.tx_buf, buffer, size);

    //发送
    APP_RTU_AT_Tx(g_app_rtu_at.tx_buf, size);
    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOGT("rtu w: %s\n", buffer);
    }
}

void APP_RTU_AT_RESET_Cfg(void)
{
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        g_app_rtu_at.tcp_code[i] = 0;
    }
}

//rtu 重启 0不重发 1支持重发
void APP_RTU_AT_RESET(uint8_t sta)
{
    char buf[16] = "AT+MREBOOT\r\n";
    if (sta == 0)
    {
        APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
    }
    else
    {
        APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
    }
}

//rtu 上电检测
void APP_RTU_AT(void)
{
    char buf[6] = "AT\r\n";
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
}

//查询TCP/IP连接状态
static void APP_RTU_AT_MIPSTATE(uint8_t chl, uint8_t rpt)
{
    char buf[32] = {0};
    sprintf(buf, "AT+MIPSTATE=%d\r\n", chl);
    if (rpt > 0)
    {
        APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
    }
    else
    {
        APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
    }
//    LOG("buf=%s\n",buf);
}

//查询网络注册状态 RTU_AT_CEREG
static void APP_RTU_AT_CEREG(void)
{
    char buf[16] = "AT+CEREG?\r\n";
    APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
}

// 数据发送格式
// 0 ASCII字符串（原始数据）
// 1 HEX字符串
// 2 带转义的字符串
// 数据接收格式
// 0 ASCII字符串（原始数据）
// 1 HEX字符串
static void APP_RTU_AT_ENCOD(void)
{
    char buf[32] = {0};
    int ret = -1;
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_rtu_at.tcp_code[i] == 0)
        {
            g_app_rtu_at.tcp_code[i] = 1;
            sprintf(buf, "AT+MIPCFG=\"encoding\",%d,1,0\r\n", i); //chl,发送格式 1,接收格式 0
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
//    LOGT("encode chl[%d]\n",g_app_rtu_at.tcp_cnt_chl);
}

void APP_RTU_AT_ENCOD_Chl(int chl)
{
    g_app_rtu_at.tcp_code[chl] = 0;
    APP_RTU_AT_ENCOD();
}

// AT+MIPOPEN=0,"TCP","112.125.89.8",47696
//任务配置  10  SOCKET任务（TCP透传/UDP透传/多通道SOCK）
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

static void APP_RTU_AT_MIPOPEN_Chl(uint8_t chl)
{
    g_app_rtu_at.tcp_cnt_chl = chl;
    char buf[RTU_TX_AT_LEN] = {0};
    sprintf(buf, "AT+MIPOPEN=%d,\"TCP\",%s,%d\r\n", chl, g_app_dtu_ip[chl].ip, g_app_dtu_ip[chl].port);
    APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
    LOGT("Reconnecting tcp[%d]:\"%s\":%d\n", chl, g_app_dtu_ip[chl].ip, g_app_dtu_ip[chl].port);
}

#if 0
//// AT+MIPMODE=0,0
//切换数据模式 RTU_AT_MIPMODE
static void APP_RTU_AT_MIPMODE(uint8_t chl, uint8_t mode)
{
    char buf[48] = {0};
    sprintf(buf, "AT+MIPMODE=%d,%d\r\n", chl, mode);
    APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
}
#endif

//  AT+MIPCLOSE=0
//  OK
//  +MIPCLOSE: 0
//关闭TCP/IP连接 RTU_AT_MIPCLOSE
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
void APP_RTU_AT_MIPCLOSE_Chl(uint8_t chl, uint8_t rpt)
{
    char buf[48] = {0};
    sprintf(buf, "AT+MIPCLOSE=%d\r\n", chl);
    if (rpt > 0)
    {
        APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
    }
    else
    {
        APP_RTU_AT_Tx((uint8_t*)buf, strlen(buf));
    }
}
//查询设备ICCID号 RTU_AT_MCCID,
void APP_RTU_AT_ICCID(void)
{
    char buf[32] = "AT+MCCID\r\n";
    APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
}

//查询设备网路信号强度
void APP_RTU_AT_CSQ(void)
{
    char buf[32] = "AT+CSQ\r\n";
    APP_RTU_AT_Tx_To((uint8_t*)buf, strlen(buf));
}

//上电TCP连接
static void APP_RTU_AT_Poweron(void)
{
    uint8_t num = 0;
    APP_RTU_AT_RESET_Cfg();
    g_app_rtu_at.poweron = 1; //rtu上电
    g_app_rtu_at.poweron_chk = RTU_AT_DET_NUM + 3;

    g_app_rtu_at.step_cfg = RTU_AT_STEP10;//查询网络注册状态
    g_app_rtu_at.step_next = 0;//下一个 步骤 序号

    memset(g_app_rtu_at.tcp_close, 0, RTU_AT_CH_SUM);
    memset(g_app_rtu_at.tcp_cnt, 1, RTU_AT_CH_SUM);
    memset(g_app_rtu_at.cmd_list, 0, 16);

    APP_RTU_AT_Ip_Get_All();
    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP10;
    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP11; //数据发送格式
    }

    for (int i = 0; i < RTU_AT_CH_SUM; i++)
    {
        if (g_app_dtu_ip[i].en > 0)
        {
            g_app_rtu_at.tcp_close[i] = 1;
            g_app_rtu_at.tcp_cnt[i] = 0;
            g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP9; //关闭TCP/IP连接
            g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP4; //建立TCP/IP连接
        }
    }

    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP6; //查询设备ICCID号 6
    g_app_rtu_at.cmd_list[num++] = RTU_AT_STEP7; //查询设备网路信号强度 7

    BSP_TIMER_Start(&g_rtu_at_timer_err);//配置超时
//    LOG_HEX(g_app_rtu_at.cmd_list,num);
}

//检测应答步骤
int APP_RTU_AT_Rec_Chk(uint8_t *buffer)
{
    int ret = -1;
    char *p = strchr((char *)buffer, '+');

    if (p != NULL)
    {
        for (int i = 0; i < RTU_AT_STEP_MAX; i++)
        {
            if (strstr(p, g_rtu_cmd_buf[i]) != NULL) //判断具体指令
            {
                ret = i;
                break;
            }
        }
    }
    return ret;
}

//检测应答结果
int APP_RTU_AT_Rec_Chk_Ret(uint8_t *buffer)
{
    int ret = -1;
    char *p = (char *)buffer;
    if (strstr((char *)p, RTU_AT_OK) != NULL) //指令结果
    {
        ret = 0;
    }
    else
    {
        p = strstr((char *)p, RTU_AT_ERR);
        if (p != NULL)
        {
            p += strlen(RTU_AT_ERR);
            ret = atoi(p);

            if (g_app_rtu_at.poweron == 2)
            {
                LOGT("err:rtu code[%d]\n", ret);
            }
        }
#if 0
        else
        {
            if (strlen(p) > 1)
            {
                LOGT("err:rtu ret:%s\n", (char *)buffer);
            }
        }
#endif
    }
    return ret;
}

// +MIPSEND: 0,10
// OK
//检测发送数据
int APP_RTU_AT_Rec_Chk_MIPSEND(uint8_t *buffer)
{
    int ret = -1;
    int chl, size;
    char *p = strstr((char *)buffer, RTU_AT_MIPSEND) + strlen(RTU_AT_MIPSEND) +2;
    if (p != NULL)
    {
        chl = atoi(p);
        p += 2;
        size = atoi(p);
    }
    LOGT("snt:chl[%d],size[%d]\n", chl, size);
    return ret;
}

//+MIPOPEN: 0,0   //连接成功
//检测建立TCP/IP连接
int APP_RTU_AT_Rec_Chk_MIPOPEN(uint8_t *buffer)
{
    int ret = -1;
    int chl = 0;
    char *p = strstr((char *)buffer, RTU_AT_MIPOPEN);
    if (p != NULL) //指令结果
    {
        p = p + strlen(RTU_AT_MIPOPEN) +2;
        chl = atoi(p);

        // 关键修复：检查通道号范围
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

    if (ret == 0 && chl >= 0 && chl < RTU_AT_CH_SUM)
    {
        g_app_rtu_at.tcp_cnt[chl] = 1;
    }
    return ret;
}

//+MIPCLOSE: 0   //关闭成功
//主动关闭TCP/IP连接   RTU_AT_MIPCLOSE
int APP_RTU_AT_Rec_Chk_MIPCLOSE(uint8_t *buffer)
{
    int ret = -1;
    char *p = strstr((char *)buffer, RTU_AT_MIPCLOSE);
    if (p != NULL) //指令结果
    {
        p = p + strlen(RTU_AT_MIPCLOSE) +2;
        ret = atoi(p);
        LOGT("IP[%d] disconnected\n", ret);
        ret = 0; //退出
        APP_RTU_Tcp_Cnt_Disconnect(ret);
    }
    return ret;
}

// +MIPSTATE: 0,"TCP","112.125.89.8",47696,"CONNECTED"
// +MIPSTATE: 1,,,,"INITIAL"
// +MIPSTATE: 2,,,,"INITIAL"
// +MIPSTATE: 3,,,,"INITIAL"
// +MIPSTATE: 4,,,,"INITIAL"
// +MIPSTATE: 5,,,,"INITIAL"

// +MIPSTATE: 0,"TCP","120.27.12.119",2040,"CONNECTED"
// +MIPSTATE: 1,"TCP","120.27.12.119",2040,"CONNECTING"
// +MIPSTATE: 2,,,,"INITIAL"
// +MIPSTATE: 3,"UDP","120.27.12.119",2040,"CONNECTED"
// +MIPSTATE: 4,,,,"INITIAL"
//查询TCP/IP连接状态
int APP_RTU_AT_Rec_Chk_MIPSTATE(uint8_t *buffer)
{
    int ret = -1;
    char *p = strstr((char *)buffer, RTU_AT_MIPSTATE) + strlen(RTU_AT_MIPSTATE) +2;
    if (p != NULL)
    {
        ret = atoi(p);//atoi 将字符串转换为整数
        p = strrchr(p, ',') + 1;//strrchr 查找最后一个逗号
        if (strstr(p, "CONNECTED") == 0)//strstr 查找字符串 如果找到返回字符串的指针 否则返回NULL
        {
            APP_RTU_AT_MIPOPEN_Chl(ret);//APP_RTU_AT_MIPOPEN_Chl 建立TCP/IP连接
            char *p1 = strrchr(p, '\"');
            if (p1 != NULL)
            {
                char err_buf[11] = {0};
                memcpy(err_buf, p + 1, p1 - p - 1);
                LOGT("IP[%d]:%s\n", ret, err_buf);
            }
        }
#if 0
        else
        {
            LOGT("IP[%d]:%s\n", ret, "CONNECTED");
        }
#endif
    }
    return ret;
}

// +MCCID: 89860818102381421186
// OK
//读取ICCID
int APP_RTU_AT_Rec_Chk_ICCID(uint8_t *buffer)
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

// +CSQ="99","23"
// OK
//查询设备网路信号强度
int APP_RTU_AT_Rec_Chk_CSQ(uint8_t *buffer)
{
    int ret = -1;
    char *p1 = strstr((char *)buffer, RTU_AT_CSQ) + strlen(RTU_AT_CSQ) +2;
    if (p1 != NULL && strlen(p1) > 1)
    {
        g_app_rtu_sim.signal = atoi(p1);
        if (g_app_rtu_sim.signal > 32)
        {
            g_app_rtu_sim.signal_per = 0;
        }
        else
        {
            g_app_rtu_sim.signal_per = g_app_rtu_sim.signal * 100 / (30);
        }
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

// AT+CEREG?
// +CEREG: 0,1
// OK

// 0：未注册，但设备现在并不在搜寻新的运营商网络
// 1：已注册本地网络
// 2：未注册，设备正在搜寻基站
// 4：未知状态
// 5：已注册，但是处于漫游状态
//查询设备网路信号强度
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
#if 0
        LOGT("rtu 4g reg [%d]\n", ret);
#endif
    }
    return ret;
}

// OK
// REBOOTING
//重启应答
uint8_t APP_RTU_AT_Rec_Chk_MREBOOT(uint8_t *buffer)
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

//at接收解析
void APP_RTU_AT_Rec(uint8_t *buffer, uint16_t len)
{
    int step = APP_RTU_AT_Rec_Chk(buffer); //应答指令解析 APP_RTU_AT_Rec_Chk 应答指令解析
    int ret = -1;
    if (BSP_CONFIG_Show_Get() == 101)
    {
        LOG("rtu r:%s\n", buffer);
        LOGT("rtu r step[%d]\n", step);
    }

    if (step < 0)
    {
        ret = APP_RTU_AT_Rec_Chk_Ret(buffer);//应答结果
        if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP9)//RTU_AT_STEP9 关闭TCP/IP连接
        {
            step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
        }
        else if (ret == 0 && g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP11)//RTU_AT_STEP11 数据发送格式
        {
            step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
        }
        else if (ret == 552) //已连接 552 TCP/IP 已被使用
        {
            if (g_app_rtu_at.cmd_list[g_app_rtu_at.step_next] == RTU_AT_STEP4)//RTU_AT_STEP4 建立TCP/IP连接
            {
                step = g_app_rtu_at.cmd_list[g_app_rtu_at.step_next];
                g_app_rtu_at.tcp_cnt[g_app_rtu_at.tcp_cnt_chl] = 1;
            }
            else if (step == RTU_AT_STEP4)//RTU_AT_STEP4 建立TCP/IP连接
            {
                g_app_rtu_at.tcp_cnt[g_app_rtu_at.tcp_cnt_chl] = 1;
            }

            LOGT("err:cnt 552\n");//错误 552 TCP/IP 已被使用
        }
        else
        {
            return;//返回
        }
    }

    switch (step)//switch 语句 根据step的值执行不同的代码
    {
    case RTU_AT_STEP0://接收数据
        break;
    case RTU_AT_STEP1: //发送数据
//        APP_RTU_AT_Rec_Chk_MIPSEND(buffer);
        break;
    case RTU_AT_STEP2: //开机打印信息
        LOGT("Network Module\n");
        APP_RTU_AT_Poweron();
        break;
    case RTU_AT_STEP3: //查询TCP/IP连接状态
        APP_RTU_AT_Rec_Chk_MIPSTATE(buffer);
        break;
    case RTU_AT_STEP4: //建立TCP/IP连接
#if 0
        if (APP_RTU_AT_Rec_Chk_MIPOPEN(buffer) == 0 || ret == 552)
        {
            BSP_TIMER_Stop(&g_rtu_at_timer);
            g_app_rtu_at.step_cfg = g_app_rtu_at.cmd_list[++g_app_rtu_at.step_next];
        }
#else
        BSP_TIMER_Stop(&g_rtu_at_timer);
        APP_RTU_AT_Rec_Chk_MIPOPEN(buffer);
        APP_RTU_AT_Rec_Cfg_Next();
#endif
        break;
    case RTU_AT_STEP5://切换数据模式.
        LOGT("RTU_AT_STEP5\n");
        break;
    case RTU_AT_STEP6://查询设备ICCID号
        BSP_TIMER_Stop(&g_rtu_at_timer);
        APP_RTU_AT_Rec_Chk_ICCID(buffer);
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    case RTU_AT_STEP7://查询设备网路信号强度
        BSP_TIMER_Stop(&g_rtu_at_timer);
        APP_RTU_AT_Rec_Chk_CSQ(buffer);
        g_app_rtu_at.step_cfg = RTU_AT_STEP_SUCCESS; //完成
        g_app_rtu_at.poweron = 2;
        BSP_TIMER_Stop(&g_rtu_at_timer_err);//配置超时
        break;
    case RTU_AT_STEP8://重启
        APP_RTU_AT_Rec_Chk_MREBOOT(buffer);
        break;
    case RTU_AT_STEP9://关闭TCP/IP连接
        BSP_TIMER_Stop(&g_rtu_at_timer);
        APP_RTU_AT_Rec_Chk_MIPCLOSE(buffer);
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    case RTU_AT_STEP10://查询网络注册状态
        if (APP_RTU_AT_Rec_Chk_CEREG(buffer) > 0) //注册成功
        {
            BSP_TIMER_Stop(&g_rtu_at_timer);
            APP_RTU_AT_Rec_Cfg_Next();
        }
        else
        {
            g_app_rtu_at.tx_repeat = 3;
        }
        break;
    case RTU_AT_STEP11://执行下一步
        BSP_TIMER_Stop(&g_rtu_at_timer);
        APP_RTU_AT_Rec_Cfg_Next();
        break;
    }
}

//连接断开
void APP_RTU_Tcp_Cnt_Disconnect(uint8_t chl)
{
    g_app_rtu_at.tcp_cnt[chl] = 0;

}

//检测到上电,但是配置没完成
void APP_RTU_AT_Config_Handle_Err(void)
{
    g_app_rtu_at.poweron = 2;
}
//检测RTU状态
int APP_RTU_AT_Ready_Chk(void)
{
    int ret = -1;
    static uint32_t tick_counter = 0; // 本地时钟计数器(100ms单位)
    tick_counter++;
    
    g_app_rtu_at.poweron_chk++;
    
    // AT层故障检测与恢复机制
    // 当网络离线且poweron=0或2时，计数连续失败检查
    if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF && 
        (g_app_rtu_at.poweron == 0 || g_app_rtu_at.poweron == 2))
    {
        g_app_rtu_at.at_fail_count++;
        
        // 达到AT失败上限，触发软重启
        if (g_app_rtu_at.at_fail_count >= AT_FAIL_LIMIT)
        {
            g_app_rtu_at.at_fail_count = 0; // 重置失败计数
            g_app_rtu_at.soft_reset_times++;
            
            LOGT("warn:AT layer detected %d consecutive failures, triggering soft reset #%d\n", 
                 AT_FAIL_LIMIT, g_app_rtu_at.soft_reset_times);
            APP_RTU_AT_RESET(0); // 软重启(AT+MREBOOT)
            
            // 如果软重启次数达到上限，触发硬重启
            if (g_app_rtu_at.soft_reset_times >= SOFT_RESET_LIMIT)
            {
                uint32_t ticks_since_last = get_ticks_since_last_reboot_at(tick_counter, g_app_rtu_at.last_hard_reboot_tick);
                if (ticks_since_last >= HARD_REBOOT_MIN_GAP_TICKS)
                {
                    LOGT("warn:AT layer triggering hard power reboot after %d soft resets (last hard reboot %d ticks ago)\n",
                         SOFT_RESET_LIMIT, ticks_since_last);
                    g_app_rtu_at.soft_reset_times = 0; // 重置软重启计数
                    g_app_rtu_at.last_hard_reboot_tick = tick_counter;
                    BSP_DTU_Power_Reboot(); // 硬件断电重启
                }
                else
                {
                    LOGT("warn:hard reboot skipped by AT layer - too soon (gap=%d ticks, need %d)\n",
                         ticks_since_last, HARD_RESET_MIN_GAP_TICKS);
                    // 不重置soft_reset_times，下次软重启后会再次尝试
                }
            }
        }
    }
    else
    {
        // 网络恢复，重置故障计数器
        if (g_app_rtu_at.at_fail_count > 0 || g_app_rtu_at.soft_reset_times > 0)
        {
            LOGT("info:AT layer recovery - network restored, resetting counters (at_fail=%d, soft_reset=%d)\n",
                 g_app_rtu_at.at_fail_count, g_app_rtu_at.soft_reset_times);
            g_app_rtu_at.at_fail_count = 0;
            g_app_rtu_at.soft_reset_times = 0;
        }
    }
    
    //dtu上电检测
    if (g_app_rtu_at.poweron == 0) //未检出到开机
    {
        if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF)
        {
            if (g_app_rtu_at.poweron_chk % 1200 == 0) //间隔120s
            {
                g_app_rtu_at.poweron_chk_th = 40;
                APP_RTU_AT_RESET(0);//重启
                LOGT("reboot:rtu\n");
            }
//            else if(g_app_rtu_at.poweron_chk==100)
//          {
//
//          }
            else if (g_app_rtu_at.poweron_chk % g_app_rtu_at.poweron_chk_th == 0)
            {
                g_app_rtu_at.poweron_chk_th = 15;
                APP_RTU_AT();
            }
        }
    }
    else if (g_app_rtu_at.poweron == 2) //上电并连接成功
    {
        if (g_app_rtu_at.poweron_chk % 100 == 0) //间隔10s  检测联网状态
        {
#if 0
            LOGT("g_app_rtu_at.poweron_chk = %d\n", g_app_rtu_at.poweron_chk);
#endif

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

    //防止编码格式不对
    if (APP_RTU_AT_Chk_Cntn_Sta() == USR_STATE_OFF && g_app_rtu_at.poweron > 0)
    {
        if (++g_app_rtu_at.net_timeout_cnt > 300) //15s不上网,更新编码格式
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
//at 配置定时回调
void APP_RTU_AT_Config_Handle(void)
{
    if (APP_RTU_AT_Ready_Chk() == 0)
    {
        switch (g_app_rtu_at.step_cfg)
        {
        case RTU_AT_STEP0://接收数据
            break;
        case RTU_AT_STEP1: //发送数据
            break;
        case RTU_AT_STEP2: //开机打印信息
            break;
        case RTU_AT_STEP3: //查询TCP/IP连接状态
            //      APP_RTU_AT_MIPSTATE(0);
            break;
        case RTU_AT_STEP4: //建立TCP/IP连接
            APP_RTU_AT_MIPOPEN();
            break;
        case RTU_AT_STEP5://切换数据模式.
            break;
        case RTU_AT_STEP6://查询设备ICCID号
            APP_RTU_AT_ICCID();
            break;
        case RTU_AT_STEP7://查询设备网路信号强度
            APP_RTU_AT_CSQ();
            break;
        case RTU_AT_STEP8://重启
            break;
        case RTU_AT_STEP9://断开连接
            APP_RTU_AT_MIPCLOSE();
            break;
        case RTU_AT_STEP10://查询网络注册状态
            APP_RTU_AT_CEREG();
            break;
        case RTU_AT_STEP11://数据发送格式
            APP_RTU_AT_ENCOD();
            break;
        case RTU_AT_STEP_FAIL: //失败
            break;
        case RTU_AT_STEP_SUCCESS: //成功
            break;
        }

        if (g_app_rtu_at.step_cfg < RTU_AT_STEP_MAX)
        {
            g_app_rtu_at.step_cfg = RTU_AT_STEP_MAX;
        }
    }
}

//rtu上电检测  0 无,1上电,2联网成功
uint8_t  APP_RTU_AT_Chk_Ready(void)
{
    return g_app_rtu_at.poweron;
}

void APP_RTU_AT_Init(void)
{
    BSP_TIMER_Init(&g_rtu_at_timer, APP_RTU_AT_timer, TIMEOUT_2S, 0);
    BSP_TIMER_Init(&g_rtu_at_config_timer, APP_RTU_AT_Config_Handle, 100, 100);
    BSP_TIMER_Start(&g_rtu_at_config_timer);
    BSP_TIMER_Init(&g_rtu_at_timer_err, APP_RTU_AT_Config_Handle_Err, TIMEOUT_10S, 0);

#ifdef USE_RTU_CACHE
    BSP_TIMER_Init(&g_timer_rtu_cache, APP_RTU_Cache_Send_Callback, TIMEOUT_200MS, TIMEOUT_200MS);
    BSP_TIMER_Start(&g_timer_rtu_cache);

    ringbuf_init(&g_rtu_rb_t, g_rtu_rb_buff, 10);
#endif
}

/*****END OF FILE****/

