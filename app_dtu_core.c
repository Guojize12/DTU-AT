#include "app_dtu_core.h"
#include "app_dtu_proto.h"
#include "app_dtu_param.h"
#include "app_dtu_upload.h"
#include "app_rtu_at.h"
#include "app_config.h"

#define APP_DTU_SIGNAL_TIMEOUT  (6200)

extern gss_device  GSS_device;
extern gss_device_alarm_stat GSS_device_alarm_stat;
static Timer g_timer_dtu = {0};

dtu_remote_def  g_dtu_remote =
{
    .uint_sn = REMOTE_SN,
    .uint_model = REMOTE_MODEL,
    .uint_ver = REMOTE_VER,
};

dtu_cmd_def g_dtu_cmd = {0};
bsp_gps_def g_gps_date = {0};
dtu_remote_cmd_def g_dtu_remote_cmd =
{
    .normal_interval = 20,
    .work_interval = 20,
};

void APP_DTU_Status_Reset(void)
{
    g_dtu_cmd.cnt_status = USR_ERROR;
    g_dtu_cmd.power_on_status = USR_ERROR;
    g_dtu_cmd.response_cmd = DTU_CNT_STEP0;
    g_dtu_cmd.send_num = 0;
    g_dtu_cmd.time_num = 0; //上传时间计时清0
    g_dtu_cmd.response_num = 0; //重置时清零
    g_dtu_cmd.gps_send_interval = 31; //定位上传间隔31s
    g_dtu_cmd.gps_enable = 0; //不支持定位功能
}

#define APP_DTU_WAITTIME 120   //单位 ms
static uint16_t  g_net_cnt_retry[DTU_CNT_STEP_MAX] = { 0, 30, 30, 30 };

uint8_t  APP_DTU_Connect_Remote_Handle(void)
{
    uint8_t ret = USR_ERROR;

    g_dtu_cmd.send_num++;
    g_dtu_cmd.send_cmd = DTU_CNT_STEP_MAX;

    if (g_dtu_cmd.cnt_status == USR_EOK) //有应答
    {
        g_dtu_cmd.cnt_status = USR_ERROR;
        g_dtu_cmd.send_num = 0;
        if (++g_dtu_cmd.response_cmd >= DTU_CNT_STEP_MAX)
        {
            ret = USR_EOK;
        }
        else
        {
            g_dtu_cmd.send_cmd = g_dtu_cmd.response_cmd;
        }
    }
    else
    {
        if (g_net_cnt_retry[DTU_CNT_STEP0] == 0)
        {
            g_net_cnt_retry[DTU_CNT_STEP0] = APP_DTU_WAITTIME;
        }
        else if (APP_RTU_AT_Chk_Ready() == 2 && g_net_cnt_retry[DTU_CNT_STEP0] == APP_DTU_WAITTIME)
        {
            g_dtu_cmd.send_cmd = DTU_CNT_STEP0;
            g_net_cnt_retry[DTU_CNT_STEP0] = 30;
            g_dtu_cmd.send_num = 0;
        }
        else if (APP_RTU_AT_Chk_Ready() == 1 && g_dtu_cmd.send_num < 50)
        {
            return ret;
        }
        else  if (g_dtu_cmd.send_num % g_net_cnt_retry[g_dtu_cmd.response_cmd] == 0)
        {
            g_dtu_cmd.send_cmd = g_dtu_cmd.response_cmd;

            if (g_dtu_cmd.response_cmd == (DTU_CNT_STEP_MAX - 1))
            {
                if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 2)
                {
                    LOGT("err:startup networking!\n");
                    ret = USR_EOK;
                    g_dtu_cmd.response_cmd = 0;
                    g_dtu_cmd.cnt_status = USR_ERROR;
                    g_dtu_cmd.send_cmd = DTU_CNT_STEP_MAX;
                    g_dtu_cmd.send_num = 0;
                }
            }
            else if (g_dtu_cmd.response_cmd == DTU_CNT_STEP0)
            {
                if (g_net_cnt_retry[DTU_CNT_STEP0] == APP_DTU_WAITTIME)
                {
                    g_net_cnt_retry[DTU_CNT_STEP0] = 30;
                    g_dtu_cmd.send_num = 0;
                }
                else if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 10)
                {
                    g_net_cnt_retry[DTU_CNT_STEP0] = 300;
                    g_dtu_cmd.send_num = 0;
                }
            }
            else
            {
                if (g_dtu_cmd.send_num / g_net_cnt_retry[g_dtu_cmd.response_cmd] > 2)
                {
                    g_dtu_cmd.send_cmd = ++g_dtu_cmd.response_cmd;
                    g_dtu_cmd.send_num = 0;
                }
            }
        }
    }

    switch (g_dtu_cmd.send_cmd)
    {
    case DTU_CNT_STEP0:
        LOGT("cnt:step0 - send heartbeat [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_Send_Hearbeat();
        break;
    case DTU_CNT_STEP1:
        LOGT("cnt:step1 - get server time [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_GetServerTime();
        break;
    case DTU_CNT_STEP2:
        LOGT("cnt:step2 - send power on data [retry:%d]\n", g_dtu_cmd.send_num);
        APP_DTU_SendDTUPowerOnData();
        break;
    case DTU_CNT_STEP3:
        APP_DTU_Send_System_config();
        g_dtu_cmd.cnt_status = USR_EOK;
        LOGT("cnt:step3 - connect complete!\n");
        break;
    default:
        break;
    }

    return ret;
}

extern uint8_t alarm_dtu_trig ;
extern uint16_t damage_degree;
extern uint16_t small_alarm_count ;
extern uint16_t big_alarm_count ;
extern int32_t position_diff;
extern uint32_t g_current_position;

void APP_DTU_Callback(void)
{
    if (g_dtu_cmd.power_on_status == USR_ERROR)
    {
        if (APP_DTU_Connect_Remote_Handle() == USR_EOK)
        {
            g_dtu_cmd.power_on_status = USR_EOK;
            LOGT("Connect to remote server complete!\n");
        }
    }
    else if (++g_dtu_cmd.response_num > APP_DTU_SIGNAL_TIMEOUT)
    {
        g_dtu_cmd.net_status = USR_STATE_OFF;

        if (g_dtu_cmd.response_num == APP_DTU_SIGNAL_TIMEOUT + 10)
        {
            LOGT("warn:dtu timeout - no response for 2min, reconnecting now...\n");

            g_app_rtu_at.poweron = 0;
            g_app_rtu_at.poweron_chk = 0;

            g_net_cnt_retry[DTU_CNT_STEP0] = 30;

            APP_DTU_Status_Reset();
            g_dtu_cmd.power_on_status = USR_ERROR;

            LOGT("reconnect:tcp reset complete, start reconnection\n");
        }
        else if ((g_dtu_cmd.response_num - APP_DTU_SIGNAL_TIMEOUT) % 300 == 0)
        {
            uint16_t retry_times = (g_dtu_cmd.response_num - APP_DTU_SIGNAL_TIMEOUT) / 300;
            LOGT("warn:reconnect retry #%d - still offline\n", retry_times);
            g_net_cnt_retry[DTU_CNT_STEP0] = 0;

            g_app_rtu_at.poweron = 0;
            g_app_rtu_at.poweron_chk = 0;
            APP_DTU_Status_Reset();
            g_dtu_cmd.power_on_status = USR_ERROR;
        }
    }
    else
    {
        g_dtu_cmd.net_status = USR_STATE_ON;

        if (alarm_dtu_trig == 1 && (GSS_device_alarm_stat.alarm == 1 || GSS_device_alarm_stat.alarm == 2))
        {
            alarm_dtu_trig = 0;
            APP_DTU_Send_DTUAlarm_sig();
        }
        else if (g_dtu_cmd.time_num % g_dtu_remote_cmd.normal_interval == 0)
        {
            static uint16_t normal_upload_counter = 0;

            if ((position_diff < -80) || (position_diff > 80))
            {
                APP_DTU_Send_DTURealTimeRecord();
                normal_upload_counter = 0;
                LOGT("send:position exceed [diff:%d]\n", position_diff);
            }
            else
            {
                normal_upload_counter++;
                if (normal_upload_counter >= 15)
                {
                    APP_DTU_Send_DTURealTimeRecord();
                    normal_upload_counter = 0;
                    LOGT("send:normal interval [30s]\n");
                }
            }
        }

        if (g_dtu_cmd.time_num % 99 == 0)
        {
            APP_DTU_Send_Hearbeat();
        }
        else if (g_dtu_cmd.time_num % 20 == 0 && g_app_rtu_sim.sta == 0)
        {
            APP_DTU_SendDTUSim();
        }

        if (g_dtu_cmd.time_num % 36000 == 0)
        {
            APP_DTU_Send_System_config();
        }
        else if (g_dtu_cmd.time_num % 3010 == 0)
        {
            APP_DTU_GetServerTime();
        }

        g_dtu_cmd.time_num++;
    }
}

void APP_DTU_Init(void)
{
    APP_RTU_AT_Init();
    APP_DTU_Status_Reset();
    APP_DTU_Remote_Head_Init();
    BSP_DELAY_MS(2000);
    BSP_TIMER_Init(&g_timer_dtu, APP_DTU_Callback, TIMEOUT_100MS, TIMEOUT_100MS);
    BSP_TIMER_Start(&g_timer_dtu);
}

void APP_DTU_Handle(void)
{
    APP_DTU_Rec_Handle();
}

void BSP_DTU_Power_Reboot(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(1000);
}