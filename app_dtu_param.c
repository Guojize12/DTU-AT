#include "app_dtu_param.h"
#include "app_dtu_proto.h"
#include "app_rtu_at.h"
#include "app_config.h"

extern gss_device  GSS_device;
extern gss_device_alarm_stat GSS_device_alarm_stat;
extern uint32_t g_current_position ;     

void APP_DTU_SendDTUPowerOnData(void)
{
    LOGT("smt:send poweron\n");
    uint8_t txBuf[100];
    uint16_t num = 23;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    txBuf[num++] = 0;
    txBuf[num++] = 0;
    txBuf[num++] = 0;
    txBuf[num++] = 0;

    uint8_t version[3];
    APP_VERSION_Get_Soft(version);
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

    APP_VERSION_Get_Hard(version);
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

    memcpy(txBuf + num, g_app_cfg.model, 20);
    num += 20;

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_POWE_ON_STATUS, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}

void APP_DTU_SendDTUSim(void)
{
    if (strlen(g_app_rtu_sim.iccid) < 10)
    {
        LOGT("err:read iccid none.\n");
        return;
    }

    LOGT("smt:sim iccid.\n");

    uint8_t txBuf[64] = {0};
    uint16_t num = 23;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    txBuf[num++] = 20;
    memcpy(txBuf + num, g_app_rtu_sim.iccid, 20);
    num += 20;
    txBuf[num++] = 0;
    txBuf[num++] = g_app_rtu_sim.signal_per;

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_SIM, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}

void APP_DTU_Cmd_Upload_Interval_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data;
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];
    g_dtu_remote_cmd.normal_interval = ie_data.u32 * 10;

    ie_data.u8_buf[3] = rxBuf[27];
    ie_data.u8_buf[2] = rxBuf[28];
    ie_data.u8_buf[1] = rxBuf[29];
    ie_data.u8_buf[0] = rxBuf[30];
    g_dtu_remote_cmd.work_interval = ie_data.u32 * 10;

    APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
    LOGT("updata interval normal[%d],work[%d]\n", g_dtu_remote_cmd.normal_interval, g_dtu_remote_cmd.work_interval);
}

static void nucRecDTUSetParaData(uint16_t cmd, uint32_t  addr, uint32_t dat, uint8_t *rxBuf, uint16_t rxLen)
{
    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;
    extern uint16_t flash_save_enable;
    extern uint16_t alarm_button_or_dwin;

    uint8_t ret = DTU_CMD_RESPONSE_SUCCESS;
    float pos_upper1, pos_lower1, pos_upper2, pos_lower2, pos_zero;
    uint32_t sig_upper1, sig_lower1, sig_upper2, sig_lower2;

    LOGT("set addr:%d --value:%d\n", addr, dat);
    switch (addr)
    {
    case 0x0001:
        APP_CONFIG_Did_Set(dat);
        break;
    case 0x0002:
        g_dtu_remote_cmd.normal_interval = dat * 10;
        break;
    case 0x0003:
        g_dtu_remote_cmd.work_interval = dat * 10;
        break;
    case 0x0005:
        break;
    case 0x0006:
        if (dat != 0) { APP_DTU_Send_System_config(); }
        return;
    case 0x0007:
        break;
    case 0x0008:
        break;
    case 0x0009:
        break;
    case 0x000A:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("system:rebooting...\n");
        BSP_DELAY_MS(100);
        BSP_CONFIG_System_Reset();
        return;
    case 0x000B:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("dtu:rebooting...\n");
        BSP_DELAY_MS(200);
        return;
    case 0x000C:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("factory:resetting...\n");
        BSP_DELAY_MS(200);
        return;
    case 0x000D:
        break;
    case 0x000E:
        break;
    case 0x000F:
        break;

    case 0x1e00:
        GSS_device.position_range_upper = dat;
        EEPROM_FLASH_WriteU32(FLASH_POS_TOP_DATA, GSS_device.position_range_upper);
        GSS_device.position_signal_upper = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);
        alarm_button_or_dwin = 1;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        pos_upper1 = (float)GSS_device.position_range_upper;
        pos_lower1 = (float)GSS_device.position_range_lower;
        sig_upper1 = GSS_device.position_signal_upper;
        sig_lower1 = GSS_device.position_signal_lower;
        if (sig_upper1 != sig_lower1)
        {
            GSS_device.position_slope = (pos_upper1 - pos_lower1) / (sig_upper1 - sig_lower1);
            GSS_device.position_offset = pos_upper1 - GSS_device.position_slope * sig_upper1;
        }
        LOGT("gss:position_range_upper=%d\n", dat);
        break;
    case 0x1e01:
        GSS_device.position_range_lower = dat;
        EEPROM_FLASH_WriteU32(FLASH_POS_BUT_DATA, GSS_device.position_range_lower);
        GSS_device.position_signal_lower = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);
        alarm_button_or_dwin = 1;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        pos_lower2 = (float)GSS_device.position_range_lower;
        pos_upper2 = (float)GSS_device.position_range_upper;
        sig_lower2 = GSS_device.position_signal_lower;
        sig_upper2 = GSS_device.position_signal_upper;
        if (sig_upper2 != sig_lower2)
        {
            GSS_device.position_slope = (pos_upper2 - pos_lower2) / (sig_upper2 - sig_lower2);
            GSS_device.position_offset = pos_upper2 - GSS_device.position_slope * sig_upper2;
        }
        LOGT("gss:position_range_lower=%d\n", dat);
        break;
    case 0x1e02:
        GSS_device.position_signal_upper = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_TOP_DATA, GSS_device.position_signal_upper);
        alarm_button_or_dwin = 1;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        LOGT("gss:position_signal_upper=%d\n", dat);
        break;
    case 0x1e03:
        GSS_device.position_signal_lower = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_SIG_BUT_DATA, GSS_device.position_signal_lower);
        alarm_button_or_dwin = 1;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        LOGT("gss:position_signal_lower=%d\n", dat);
        break;
    case 0x1e04:
        GSS_device.total_length = dat;
        EEPROM_FLASH_WriteU16(TOTAL_LEN_1, GSS_device.total_length);
        LOGT("gss:total_length=%d\n", dat);
        break;
    case 0x1e05:
        MEAN_DEVIATION_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_MEAN_DEVIATION_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:MEAN_DEVIATION_THRESHOLD=%f\n", MEAN_DEVIATION_THRESHOLD);
        break;
    case 0x1e06:
        SENSOR_DEVIATION_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_SENSOR_DEVIATION_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:SENSOR_DEVIATION_THRESHOLD=%f\n", SENSOR_DEVIATION_THRESHOLD);
        break;
    case 0x1e07:
        VARIANCE_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_VARIANCE_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:VARIANCE_THRESHOLD=%f\n", VARIANCE_THRESHOLD);
        break;
    case 0x1e08:
        TREND_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_TREND_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:TREND_THRESHOLD=%f\n", TREND_THRESHOLD);
        break;
    case 0x1e09:
        DEFECT_SCORE_THRESHOLD = (float)dat / 10.0f;
        EEPROM_FLASH_WriteU16(FLASH_DEFECT_SCORE_THRESHOLD, (uint16_t)(dat));
        LOGT("gss:DEFECT_SCORE_THRESHOLD=%f\n", DEFECT_SCORE_THRESHOLD);
        break;
    case 0x1e0a:
        if (dat == 0)
        {
            flash_save_enable = 0;
            EEPROM_FLASH_WriteU16(FLASH_SAVE_ENABLE, 0);
        }
        else
        {
            flash_save_enable = 1;
            EEPROM_FLASH_WriteU16(FLASH_SAVE_ENABLE, 1);
        }
        break;
    case 0x1e0b:
        break;
    case 0x1e0c:
        GSS_device.Threshold_set1 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_1, GSS_device.Threshold_set1);
        break;
    case 0x1e0d:
        GSS_device.Threshold_set2 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_2, GSS_device.Threshold_set2);
        break;
    case 0x1e0f:
        GSS_device.Threshold_set3 = dat;
        EEPROM_FLASH_WriteU16(FLASH_THRESHOLD_3, GSS_device.Threshold_set3);
        break;
    case 0x1e10:
        GSS_device.Threshold_set4 = dat;
        break;
    case 0x1e11:
        GSS_device.position_zero_point = g_current_position;
        EEPROM_FLASH_WriteU32(FLASH_POSITION_ZERO_POINT, GSS_device.position_zero_point);
        alarm_button_or_dwin = 0;
        EEPROM_FLASH_WriteU16(FLASH_BUTTON_OR_DWIN, alarm_button_or_dwin);
        pos_zero = (float)GSS_device.position_zero_point;
        if (pos_zero != 0)
        {
            GSS_device.position_slope = (pos_zero - pos_zero) / (pos_zero - pos_zero);
            GSS_device.position_offset = pos_zero - GSS_device.position_slope * pos_zero;
        }
        break;
    default:
        ret = DTU_CMD_RESPONSE_FAIL;
        LOGT("error:unknown addr:0x%04X\n", addr);
        break;
    }

    APP_DTU_Response_Result(cmd, ret, rxBuf, rxLen);
}

void APP_DTU_Cmd_Config_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    uint32_t addr, data;
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];
    addr = ie_data.i32;
    ie_data.u8_buf[3] = rxBuf[27];
    ie_data.u8_buf[2] = rxBuf[28];
    ie_data.u8_buf[1] = rxBuf[29];
    ie_data.u8_buf[0] = rxBuf[30];
    data = ie_data.i32;
    nucRecDTUSetParaData(cmd, addr, data, rxBuf, rxLen);
}

void APP_DTU_Cmd_Config_Get(uint16_t cmd, uint32_t addr, uint8_t pid)
{
    IEEE754 ie_data = {0};
    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    ie_data.u32 = addr;
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    txBuf[num++] = 1;

    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;

    switch (addr)
    {
    case 0x0001: ie_data.u32 = g_app_cfg.did; break;
    case 0x0002: ie_data.u32 = g_dtu_remote_cmd.normal_interval / 10; break;
    case 0x0003: ie_data.u32 = g_dtu_remote_cmd.work_interval / 10; break;
    case 0x0005: ie_data.u32 = 0; break;
    case 0x0007: ie_data.u32 = 0; break;
    case 0x0008: ie_data.u32 = 0; break;
    case 0x0009: ie_data.u32 = 0; break;
    case 0x000D: ie_data.u32 = 0; break;
    case 0x000E: ie_data.u32 = 0; break;
    case 0x000F: ie_data.u32 = 0; break;

    case 0x1e00: ie_data.u32 = GSS_device.position_range_upper; break;
    case 0x1e01: ie_data.u32 = GSS_device.position_range_lower; break;
    case 0x1e02: ie_data.u32 = GSS_device.position_signal_upper; break;
    case 0x1e03: ie_data.u32 = GSS_device.position_signal_lower; break;
    case 0x1e04: ie_data.u32 = GSS_device.total_length; break;
    case 0x1e05: ie_data.u32 = (uint32_t)(MEAN_DEVIATION_THRESHOLD * 10); break;
    case 0x1e06: ie_data.u32 = (uint32_t)(SENSOR_DEVIATION_THRESHOLD * 10); break;
    case 0x1e07: ie_data.u32 = (uint32_t)(VARIANCE_THRESHOLD * 10); break;
    case 0x1e08: ie_data.u32 = (uint32_t)(TREND_THRESHOLD * 10); break;
    case 0x1e09: ie_data.u32 = (uint32_t)(DEFECT_SCORE_THRESHOLD * 10); break;
    case 0x1e0a: ie_data.u32 = 0; break;
    case 0x1e0b: ie_data.u32 = 0; break;
    case 0x1e0c: ie_data.u32 = GSS_device.Threshold_set1; break;
    case 0x1e0d: ie_data.u32 = GSS_device.Threshold_set2; break;
    case 0x1e0f: ie_data.u32 = GSS_device.Threshold_set3; break;
    case 0x1e10: ie_data.u32 = GSS_device.Threshold_set4; break;
    case 0x1e11: ie_data.u32 = 0; break;
    case 0x1e12: ie_data.u32 = 0; break;
    default:
        ie_data.u32 = 0;
        LOGT("warn:unknown addr:0x%04X for get\n", addr);
        break;
    }

    LOGT("get addr:0x%04X --value:%d\n", addr, ie_data.u32);

    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, pid);

    uint16_t crc16 = bsp_crc16(txBuf + 23, (num - 23));
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
    LOGT("send config get to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}

void APP_DTU_Cmd_Config_Get_Response(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    ie_data.u8_buf[3] = rxBuf[23];
    ie_data.u8_buf[2] = rxBuf[24];
    ie_data.u8_buf[1] = rxBuf[25];
    ie_data.u8_buf[0] = rxBuf[26];
    APP_DTU_Cmd_Config_Get(cmd, ie_data.u32, rxBuf[20]);
}

void APP_DTU_Cmd_Ip_Set(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data = {0};
    app_dtu_ip_def dtu_ip = {0};

    ie_data.u8_buf[3] = rxBuf[28];
    ie_data.u8_buf[2] = rxBuf[29];
    ie_data.u8_buf[1] = rxBuf[30];
    ie_data.u8_buf[0] = rxBuf[31];
    dtu_ip.port = ie_data.i32;
    ie_data.u8_buf[3] = rxBuf[24];
    ie_data.u8_buf[2] = rxBuf[25];
    ie_data.u8_buf[1] = rxBuf[26];
    ie_data.u8_buf[0] = rxBuf[27];

    memcpy(dtu_ip.ip, rxBuf + 32, ie_data.i32);

    APP_RTU_AT_Ip_Set(rxBuf[23] - 1, dtu_ip.ip, dtu_ip.port, 1);
}

void APP_DTU_Cmd_Ip_Get(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    if (rxBuf[23] == 0 || rxBuf[23] > 4)
    {
        LOG("err:mt get ip!\n");
        return;
    }
    uint8_t chl = rxBuf[23] - 1;

    IEEE754 ie_data = {0};
    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    txBuf[num++] = 0;
    txBuf[num++] = rxBuf[23];

    ie_data.u32 = strlen(g_app_dtu_ip[chl].ip);
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    ie_data.u32 = g_app_dtu_ip[chl].port;
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    memcpy(txBuf + num, g_app_dtu_ip[chl].ip, strlen(g_app_dtu_ip[chl].ip));
    num += strlen(g_app_dtu_ip[chl].ip);

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, rxBuf[20]);

    uint16_t crc16 = bsp_crc16(txBuf + 23, (num - 23));
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}

void APP_DTU_Send_System_config(void)
{
    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;

    uint8_t txBuf[256] = {0};
    uint16_t num = 23;
    IEEE754 aucData;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    txBuf[num++] = 0;

    aucData.u32 = GSS_device.position_range_upper;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.u32 = GSS_device.position_range_lower;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.u32 = GSS_device.position_signal_upper;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.u32 = GSS_device.position_signal_lower;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.flt = GSS_device_alarm_stat.position_data_real;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.u32 = GSS_device_alarm_stat.position_data_ad;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    txBuf[num++] = (GSS_device.total_length >> 8) & 0xFF;
    txBuf[num++] = GSS_device.total_length & 0xFF;

    aucData.flt = MEAN_DEVIATION_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.flt = SENSOR_DEVIATION_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.flt = VARIANCE_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.flt = TREND_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    aucData.flt = DEFECT_SCORE_THRESHOLD;
    txBuf[num++] = aucData.u8_buf[3];
    txBuf[num++] = aucData.u8_buf[2];
    txBuf[num++] = aucData.u8_buf[1];
    txBuf[num++] = aucData.u8_buf[0];

    txBuf[num++] = 0;
    txBuf[num++] = 0;

    txBuf[num++] = (GSS_device.Threshold_set1 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set1 & 0xFF;

    txBuf[num++] = (GSS_device.Threshold_set2 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set2 & 0xFF;

    txBuf[num++] = (GSS_device.Threshold_set3 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set3 & 0xFF;

    txBuf[num++] = (GSS_device.Threshold_set4 >> 8) & 0xFF;
    txBuf[num++] = GSS_device.Threshold_set4 & 0xFF;

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_CONFIG_INFO, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

    LOGT("send config info to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}