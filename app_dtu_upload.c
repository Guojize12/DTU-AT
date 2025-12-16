#include "app_dtu_upload.h"
#include "app_dtu_proto.h"
#include "app_config.h"

extern gss_device  GSS_device;
extern gss_device_alarm_stat GSS_device_alarm_stat;
extern uint16_t damage_degree;
extern uint16_t small_alarm_count ;
extern uint16_t big_alarm_count ;
extern AlarmInfo alarm_info_max;
extern uint32_t g_current_position ;

extern float MEAN_DEVIATION_THRESHOLD;
extern float SENSOR_DEVIATION_THRESHOLD;
extern float VARIANCE_THRESHOLD;
extern float trend_deviation;
extern float DEFECT_SCORE_THRESHOLD;
extern float filtered_value11;

extern bsp_dat_def Value_real[30];
extern uint16_t g_dwin_display_data[4];

void APP_DTU_Send_DTURealTimeRecord(void)
{
    uint8_t txBuf[1024] = {0};
    uint16_t num = 23;
    uint16_t pack_len = 1;
    IEEE754 aucData1 ;
    txBuf[num++] = 0;
    txBuf[num++] = pack_len >> 8;
    txBuf[num++ ] = pack_len;

    for (u8 i = 0; i < pack_len; i++)
    {
        BSP_RTC_Get(&g_bsp_rtc);
        txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
        txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
        txBuf[num++] = g_bsp_rtc.month;
        txBuf[num++] = g_bsp_rtc.day;
        txBuf[num++] = g_bsp_rtc.hour;
        txBuf[num++] = g_bsp_rtc.minute;
        txBuf[num++] = g_bsp_rtc.second;

        aucData1.flt = (GSS_device.position_data_real < 0) ? -GSS_device.position_data_real : GSS_device.position_data_real;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.u32 = GSS_device.position_data_ad;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = (GSS_device.real_speed < 0) ? -GSS_device.real_speed : GSS_device.real_speed;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[0] >> 8;
        txBuf[num++] = g_dwin_display_data[0] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[0] * 3300.0f) / 4095.0f;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[1] >> 8;
        txBuf[num++] = g_dwin_display_data[1] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[1] * 3300.0f) / 4095.0f;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[2] >> 8;
        txBuf[num++] = g_dwin_display_data[2] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[2] * 3300.0f) / 4095.0f;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = g_dwin_display_data[3] >> 8;
        txBuf[num++] = g_dwin_display_data[3] & 0xff;

        aucData1.flt = ((float)g_dwin_display_data[3] * 3300.0f) / 4095.0f;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = alarm_info_max.type;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = GSS_device.alarm >> 8;
        txBuf[num++] = GSS_device.alarm & 0xff;

        txBuf[num++] = small_alarm_count % 2500 >> 8;
        txBuf[num++] = small_alarm_count % 2500 & 0xff;

        txBuf[num++] = big_alarm_count % 2500 >> 8;
        txBuf[num++] = big_alarm_count % 2500 & 0xff;

        txBuf[num++] = GSS_device.total_length >> 8;
        txBuf[num++] = GSS_device.total_length & 0xff;

        txBuf[num++] = damage_degree >> 8;
        txBuf[num++] = damage_degree & 0xff;

        txBuf[num++] = GSS_device.max_damage_position >> 8;
        txBuf[num++] = GSS_device.max_damage_position & 0xff;

        aucData1.flt = MEAN_DEVIATION_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = SENSOR_DEVIATION_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = VARIANCE_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = trend_deviation;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = DEFECT_SCORE_THRESHOLD;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = filtered_value11;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];
    }
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_DATA_UPLOAD, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

    LOGT("send real-time data to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}

void APP_DTU_Send_DTUAlarm_sig(void)
{
    uint8_t txBuf[1024] = {0};
    uint16_t num = 23;
    IEEE754 aucData1 ;

    BSP_RTC_Get(&g_bsp_rtc);
    txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8;
    txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
    txBuf[num++] = g_bsp_rtc.month;
    txBuf[num++] = g_bsp_rtc.day;
    txBuf[num++] = g_bsp_rtc.hour;
    txBuf[num++] = g_bsp_rtc.minute;
    txBuf[num++] = g_bsp_rtc.second;

    LOGT("----------send alarm time: %04d-%02d-%02d %02d:%02d:%02d\n",
         2000 + g_bsp_rtc.year,
         g_bsp_rtc.month,
         g_bsp_rtc.day,
         g_bsp_rtc.hour,
         g_bsp_rtc.minute,
         g_bsp_rtc.second);

    txBuf[num++] = GSS_device_alarm_stat.alarm;

    aucData1.flt = GSS_device_alarm_stat.position_data_real;
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    aucData1.u32 = GSS_device_alarm_stat.position_data_ad = g_current_position;
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    float alarm_speed = (GSS_device_alarm_stat.real_speed < 0) ? -GSS_device_alarm_stat.real_speed : GSS_device_alarm_stat.real_speed;
    aucData1.flt = alarm_speed / 50;
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[0] >> 8;
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[0] & 0xff;

    GSS_device_alarm_stat.hall_v[0] = (uint32_t)((GSS_device_alarm_stat.hall_ad[0] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[0];
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[1] >> 8;
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[1] & 0xff;

    GSS_device_alarm_stat.hall_v[1] = (uint32_t)((GSS_device_alarm_stat.hall_ad[1] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[1];
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[2] >> 8;
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[2] & 0xff;

    GSS_device_alarm_stat.hall_v[2] = (uint32_t)((GSS_device_alarm_stat.hall_ad[2] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[2];
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    txBuf[num++] = GSS_device_alarm_stat.hall_ad[3] >> 8;
    txBuf[num++] = GSS_device_alarm_stat.hall_ad[3] & 0xff;

    GSS_device_alarm_stat.hall_v[3] = (uint32_t)((GSS_device_alarm_stat.hall_ad[3] * 3300) / 4095);
    aucData1.flt = GSS_device_alarm_stat.hall_v[3];
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    aucData1.flt = GSS_device_alarm_stat.degree_of_damage = GSS_device_alarm_stat.alarm;
    txBuf[num++] = aucData1.u8_buf[3];
    txBuf[num++] = aucData1.u8_buf[2];
    txBuf[num++] = aucData1.u8_buf[1];
    txBuf[num++] = aucData1.u8_buf[0];

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_ALARM_UPLOAD, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

    LOGT("send alarm data to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}