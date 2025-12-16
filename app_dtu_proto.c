#include "app_dtu_proto.h"
#include "app_rtu_at.h"
#include "app_dtu_param.h"  /* 补充声明，避免隐式声明警告 */
#include "app_config.h"
#include "app_dtu_reconnect.h"  // 新增：应答时通知重连模块

#define APP_DTU_UART          BSP_UART3
#define APP_DTU_UART_BUF      g_uart_buf[APP_DTU_UART]

static app_rtu_rx_def g_dtu_rx; // 多通道接收数据

void APP_DTU_Send_Buf(uint8_t *buffer, uint16_t size)
{
    BSP_UART_Transmit(APP_DTU_UART, buffer, size);
}

void APP_DTU_Send(uint8_t *buffer, uint16_t size)
{
    APP_RTU_AT_Tx_Chl(0, buffer, size);
}

void APP_DTU_Head_Packing(uint8_t type, uint8_t *txBuf, uint16_t not_head_len, uint16_t cmd, uint8_t pid)
{
    memcpy(txBuf, g_dtu_cmd.head, 20);
    txBuf[1] = type;
    txBuf[2] = (uint8_t)(not_head_len >> 8);
    txBuf[3] = (uint8_t)not_head_len & 0xFF;
    txBuf[17] = (uint8_t)(cmd >> 8);
    txBuf[18] = (uint8_t)cmd & 0xFF;
    txBuf[20] = pid;
    uint16_t crc16 = bsp_crc16((uint8_t*)txBuf, 21);
    txBuf[21] = (uint8_t)crc16 & 0xFF;
    txBuf[22] = (uint8_t)(crc16 >> 8);
}

void APP_DTU_Response_Result(uint16_t cmd, uint8_t state, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 1, cmd, rxBuf[20]);
    txBuf[23] = state;
    uint16_t crc16 = bsp_crc16(txBuf + 23, 1);
    txBuf[24] = (uint8_t)crc16 & 0xFF;
    txBuf[25] = (uint8_t)(crc16 >> 8);
    APP_DTU_Send(txBuf, 26);
}

void APP_DTU_Send_Hearbeat(void)
{
    LOGT("smt:ht\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_HEARTBEAT, 0);
    APP_DTU_Send(txBuf, 23);
    if (g_dtu_cmd.net_status == USR_STATE_OFF)
    {
        LOGT("smt:ht,no net to cnt..\n");
    }
}

void APP_DTU_GetServerTime(void)
{
    LOGT("smt:get time\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_TIMESYNC, 0);
    APP_DTU_Send(txBuf, 23);
}

void APP_DTU_Response_Hearbeat(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[64] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 0, cmd, rxBuf[20]);
    APP_DTU_Send(txBuf, 23);
}

void APP_DTU_TimeSync_Set(uint8_t *rxBuf, uint16_t rxLen)
{
    LOGT("rtc %d-%d\n", rxBuf[23], rxBuf[24]);
    uint16_t ttt = (rxBuf[23] * 256) + rxBuf[24];
    if (ttt < 100) { ttt += 2000; }
    g_bsp_rtc.year = ttt % 100;
    g_bsp_rtc.month = rxBuf[25];
    g_bsp_rtc.day = rxBuf[26];
    g_bsp_rtc.hour = rxBuf[27];
    g_bsp_rtc.minute = rxBuf[28];
    g_bsp_rtc.second = rxBuf[29];
    if (ttt > 2019) { BSP_RTC_Set(g_bsp_rtc); }
    LOGT("timing:%04d-%02d-%02d %02d:%02d:%02d\n",
         (ttt > 100 ? ttt : 2000 + g_bsp_rtc.year),
         g_bsp_rtc.month, g_bsp_rtc.day,
         g_bsp_rtc.hour, g_bsp_rtc.minute, g_bsp_rtc.second);
}

void APP_DTU_Gps_Check(uint8_t *rxBuf, uint16_t rxLen)
{
    if (strstr((char *)APP_DTU_UART_BUF.rxBuf, "$GNRMC") != NULL)
    {
        /* g_gps_date 由 app_dtu.c 定义，此处通过 extern 在头文件声明 */
        g_dtu_cmd.gps_status = bsp_gps_parse((char*)rxBuf, &g_gps_date);
        g_dtu_cmd.gps_enable = 1;
    }
    else if (strstr((char *)rxBuf, "$GNGGA") != NULL)
    {
    }
}

static int APP_DTU_Remote_Check_Head(uint8_t *rxBuf, uint16_t rxLen)
{
    int ret = 0;
    uint16_t crc16 = bsp_crc16((uint8_t*)rxBuf, 21);
    if ((((uint8_t)crc16 & 0xFF) == rxBuf[21]) &&
        ((uint8_t)(crc16 >> 8) == rxBuf[22]))
    {
        ret = 1;
    }
    return ret;
}

static int APP_DTU_Remote_Check_Body(void)
{
    int ret = 0;
    uint16_t len = g_dtu_rx.rxBuf[2];
    len = (len << 8) + g_dtu_rx.rxBuf[3];

    int deal = 0;
    uint16_t dtu_remote_index = 23 + len;
    if (len > 0)
    {
        uint16_t crc16 = bsp_crc16(g_dtu_rx.rxBuf + 23, len);
        if ((((uint8_t)crc16 & 0xFF) == g_dtu_rx.rxBuf[len + 23]) &&
            ((uint8_t)(crc16 >> 8) == g_dtu_rx.rxBuf[len + 23 + 1]))
        {
            deal = 1;
            dtu_remote_index += 2;
        }
        else
        {
            LOGT("err:body crc [0x%X]\n", crc16);
            LOG_HEX(g_dtu_rx.rxBuf, (g_dtu_rx.rxLen > 50) ? 50 : g_dtu_rx.rxLen);
        }
    }
    else
    {
        deal = 1;
    }

    if (deal > 0)
    {
        g_dtu_cmd.cmd.u8_buf[1] = g_dtu_rx.rxBuf[17];
        g_dtu_cmd.cmd.u8_buf[0] = g_dtu_rx.rxBuf[18];
        g_dtu_cmd.pid = g_dtu_rx.rxBuf[20];

        if (g_dtu_rx.rxBuf[1] == 'R')
        {
            APP_DTU_Parse_Read(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }
        else if (g_dtu_rx.rxBuf[1] == 'W')
        {
            APP_DTU_Parse_Write(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }

        if (g_dtu_rx.rxLen > dtu_remote_index)
        {
            g_dtu_rx.rxLen -= dtu_remote_index;
            memcpy(g_dtu_rx.rxBuf, g_dtu_rx.rxBuf + dtu_remote_index, g_dtu_rx.rxLen);
            ret = 1;
        }
    }

    return ret;
}

int APP_DTU_Remote_Check(void)
{
    int ret = 0;
    while (g_dtu_rx.rxLen > 22)
    {
        ret = APP_DTU_Remote_Check_Head(g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        if (ret == 1)
        {
            if (APP_DTU_Remote_Check_Body() == 0)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    return ret;
}

void APP_DTU_Rec_Handle(void)
{
    if (BSP_UART_Rec_Read(APP_DTU_UART) == USR_EOK)
    {
        if (BSP_CONFIG_Show_Get() == 50)
        {
            LOGT("rtu rx[%d]: \n", APP_DTU_UART_BUF.rxLen);
            LOG_HEX(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        }
        g_dtu_rx = APP_RTU_AT_Rx_Chl(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        if (g_dtu_rx.rxLen > 0)
        {
            APP_DTU_Remote_Check();
        }
    }
}

void APP_DTU_Parse_Read(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    switch (cmd)
    {
    case DTU_CMD_DEVICE_HEARTBEAT:
        if (rxBuf[20] == 99)
        {
            g_dtu_cmd.cnt_status = USR_EOK;
            g_dtu_cmd.response_num = 0;
            g_dtu_cmd.net_status = USR_STATE_ON;
            DTU_Reconnect_OnAck();  // 新增：应答通知
            if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:ht\n"); }
        }
        else
        {
            LOGT("err:rmt ht[%d]\n", rxBuf[20]);
            LOG_HEX(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        }
        break;
    case DTU_CMD_DEVICE_TIMESYNC:
        g_dtu_cmd.response_num = 0;
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        g_dtu_cmd.cnt_status = USR_EOK;
        DTU_Reconnect_OnAck();      // 新增：应答通知
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:time sync\n"); }
        break;
    case DTU_CMD_DEVICE_POWE_ON_STATUS:
        g_dtu_cmd.cnt_status = USR_EOK;
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();      // 新增：应答通知
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:po\n"); }
        break;
    case DTU_CMD_SERVER_GATEWAY:
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();      // 新增：应答通知
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:gateway[%d]\n", rxBuf[23]); }
        break;
    case DTU_CMD_DEVICE_SIM:
        g_app_rtu_sim.sta = 1;
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();      // 新增：应答通知
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:sim info\n"); }
        break;
    case DTU_CMD_DEVICE_GPS:
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();      // GPS信息也视为链路活跃
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("rmt:gps info\n"); }
        break;
    case DTU_CMD_SERVER_GSS_DATA_UPLOAD:
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();
        LOGT("rmt:rtd gss\n");
        break;
    case DTU_CMD_SERVER_GSS_ALARM_UPLOAD:
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();
        LOGT("rmt:ent gss\n");
        break;
    case DTU_CMD_SERVER_GSS_CONFIG_INFO:
        g_dtu_cmd.response_num = 0;
        DTU_Reconnect_OnAck();
        LOGT("rmt:config info ack\n");
        break;
    default:
        break;
    }
}

void APP_DTU_Parse_Write(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    switch (cmd)
    {
    case DTU_CMD_SERVER_HEARTBEAT:
        APP_DTU_Response_Hearbeat(cmd, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:ht\n"); }
        break;
    case DTU_CMD_SERVER_TIMESYNC:
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:time sync\n"); }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_FREQUENCY:
        APP_DTU_Cmd_Upload_Interval_Set(cmd, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:up interval\n"); }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_SET:
        APP_DTU_Cmd_Ip_Set(cmd, rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:ip set\n"); }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_GET:
        APP_DTU_Cmd_Ip_Get(cmd, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:ip get\n"); }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_SET:
        APP_DTU_Cmd_Config_Set(cmd, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOGT("wmt:cfg set\n"); }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_GET:
        APP_DTU_Cmd_Config_Get_Response(cmd, rxBuf, rxLen);
        DTU_Reconnect_OnAck();
        if (BSP_CONFIG_Show_Get() == 52) { LOG("wmt:cfg get\n"); }
        break;
    case DTU_CMD_SERVER_DEVICE_OTA:
        LOGT("ota:remote upgrade triggered\n");
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("ota:enter boot mode in 1s...\n");
        LOGT("ota:set boot flag to 0x%08X\n", OTA_FLAG_MAGIC_NUMBER);
        EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, OTA_FLAG_MAGIC_NUMBER);
        LOGT("ota:rebooting to bootloader...\n");
        BSP_CONFIG_System_Reset();
        break;
    default:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_PARSE_FAIL, rxBuf, rxLen);
        break;
    }
}

void APP_DTU_Remote_Head_Init(void)
{
    g_dtu_cmd.head[0] = '$';
    g_dtu_cmd.head[4] = ((g_dtu_remote.uint_sn / 1000000) % 10) +0x30;
    g_dtu_cmd.head[5] = ((g_dtu_remote.uint_sn / 100000) % 10) +0x30;
    g_dtu_cmd.head[6] = ((g_dtu_remote.uint_sn / 10000) % 10) +0x30;
    g_dtu_cmd.head[7] = ((g_dtu_remote.uint_sn / 1000) % 10) +0x30;
    g_dtu_cmd.head[8] = ((g_dtu_remote.uint_sn / 100) % 10) +0x30;
    g_dtu_cmd.head[9] = ((g_dtu_remote.uint_sn / 10) % 10) +0x30;
    g_dtu_cmd.head[10] = (g_dtu_remote.uint_sn % 10) +0x30;
    g_dtu_cmd.head[11] = ((g_app_cfg.did / 10000) % 10) +0x30;
    g_dtu_cmd.head[12] = ((g_app_cfg.did / 1000) % 10) +0x30;
    g_dtu_cmd.head[13] = ((g_app_cfg.did / 100) % 10) +0x30;
    g_dtu_cmd.head[14] = ((g_app_cfg.did / 10) % 10) +0x30;
    g_dtu_cmd.head[15] = (g_app_cfg.did % 10) +0x30;
    g_dtu_cmd.head[16] = g_dtu_remote.uint_ver;
    g_dtu_cmd.head[19] = g_dtu_remote.uint_model;
}