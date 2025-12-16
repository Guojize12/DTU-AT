#include "app_dtu_reconnect.h"
#include "app_dtu_core.h"
#include "app_dtu_proto.h"
#include "app_config.h"

// 全局对象
dtu_reconnect_t g_dtu_reconnect = {
    .offline_timeout_ticks        = 900,  // 90s无应答判定离线
    .initial_retry_delay_ticks    = 30,   // 3s 首次重试
    .backoff_multiplier           = 2,    // 指数退避 *2
    .max_retry_delay_ticks        = 300,  // 30s 最大退避
    .max_retries_before_hw_reboot = 3,    // 连续3次失败后硬件重启
    .max_total_hw_reboots         = 3,    // 一个会话最多3次硬件重启
    .reboot_cooldown_ticks        = 600,  // 60s 冷却
    .target_channel               = 0,    // 业务通道0
    .state                        = DTU_RECON_IDLE,
    .next_attempt_tick            = 0,
    .current_delay_ticks          = 30,
    .attempt_count                = 0,
    .hw_reboot_count              = 0,
    .tick_100ms                   = 0,
};

static inline uint16_t clamp_u16(uint16_t v, uint16_t mn, uint16_t mx) {
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

void DTU_Reconnect_Init(void)
{
    g_dtu_reconnect.state = DTU_RECON_IDLE;
    g_dtu_reconnect.next_attempt_tick = 0;
    g_dtu_reconnect.current_delay_ticks = g_dtu_reconnect.initial_retry_delay_ticks;
    g_dtu_reconnect.attempt_count = 0;
    g_dtu_reconnect.hw_reboot_count = 0;
    g_dtu_reconnect.tick_100ms = 0;
}

void DTU_Reconnect_OnAck(void)
{
    // 任何服务器应答，认为链路恢复
    g_dtu_cmd.net_status = USR_STATE_ON;
    g_dtu_reconnect.state = DTU_RECON_IDLE;
    g_dtu_reconnect.attempt_count = 0;
    g_dtu_reconnect.current_delay_ticks = g_dtu_reconnect.initial_retry_delay_ticks;
    // response_num 已在解析处清零
}

void DTU_Reconnect_OnTcpDisconnect(uint8_t ch)
{
    if (ch >= RTU_AT_CH_SUM) return;
    if (g_dtu_cmd.power_on_status != USR_EOK) {
        // 上电阶段的断开由原上电状态机处理
        return;
    }
    // 标记离线并计划重连
    g_dtu_cmd.net_status = USR_STATE_OFF;
    g_dtu_reconnect.state = DTU_RECON_OFFLINE;
    g_dtu_reconnect.attempt_count = 0;
    g_dtu_reconnect.current_delay_ticks = g_dtu_reconnect.initial_retry_delay_ticks;
    g_dtu_reconnect.next_attempt_tick = g_dtu_reconnect.tick_100ms + g_dtu_reconnect.current_delay_ticks;

    // 提前将RTU流程置为未上电状态，交由本模块重新发起
    g_app_rtu_at.poweron = 0;
    g_app_rtu_at.poweron_chk = 0;

    LOGT("reconnect: URC disconnect on chl[%d], schedule retry in %d ticks\n",
         ch, g_dtu_reconnect.current_delay_ticks);
}

static void dtu_reconnect_try_once(void)
{
    // 如果RTU还没上电，先发起上电流程
    if (APP_RTU_AT_Chk_Ready() == 0) {
        LOGT("reconnect: AT poweron start\n");
        APP_RTU_AT_Poweron();
        return;
    }

    // RTU已准备好（poweron==2），发起通道重连
    if (APP_RTU_AT_Chk_Ready() == 2) {
        LOGT("reconnect: tcp opening on chl[%d]\n", g_dtu_reconnect.target_channel);
        APP_RTU_AT_MIPOPEN_Chl(g_dtu_reconnect.target_channel);
        // 主动发一个心跳，促使服务器快速应答以判定恢复
        APP_DTU_Send_Hearbeat();

        // 累计尝试次数与下一次退避延迟
        g_dtu_reconnect.attempt_count++;
        uint32_t next = g_dtu_reconnect.current_delay_ticks * g_dtu_reconnect.backoff_multiplier;
        g_dtu_reconnect.current_delay_ticks = clamp_u16((uint16_t)next, g_dtu_reconnect.initial_retry_delay_ticks, g_dtu_reconnect.max_retry_delay_ticks);
        g_dtu_reconnect.next_attempt_tick = g_dtu_reconnect.tick_100ms + g_dtu_reconnect.current_delay_ticks;

        LOGT("reconnect: attempt #%d, next in %d ticks\n",
             g_dtu_reconnect.attempt_count, g_dtu_reconnect.current_delay_ticks);
    }
}

static void dtu_reconnect_hw_reboot(void)
{
    if (g_dtu_reconnect.hw_reboot_count >= g_dtu_reconnect.max_total_hw_reboots) {
        LOGT("reconnect: max HW reboots reached (%d), stop rebooting\n", g_dtu_reconnect.hw_reboot_count);
        return;
    }

    LOGT("reconnect: HW reboot DTU (PB2) ...\n");
    BSP_DTU_Power_Reboot();  // PB2控制DTU电源

    g_dtu_reconnect.hw_reboot_count++;
    g_dtu_reconnect.attempt_count = 0;  // 重启后重试计数清零
    g_dtu_reconnect.current_delay_ticks = g_dtu_reconnect.initial_retry_delay_ticks;
    g_dtu_reconnect.state = DTU_RECON_REBOOT_COOLDOWN;
    g_dtu_reconnect.next_attempt_tick = g_dtu_reconnect.tick_100ms + g_dtu_reconnect.reboot_cooldown_ticks;

    // 让AT栈回到未上电状态，由冷却结束后重新发起
    g_app_rtu_at.poweron = 0;
    g_app_rtu_at.poweron_chk = 0;
}

void DTU_Reconnect_Tick(void)
{
    g_dtu_reconnect.tick_100ms++;

    // 初次连接阶段按老流程
    if (g_dtu_cmd.power_on_status == USR_ERROR) return;

    // 在线路径：检测是否“长时间无应答”-> 转入离线
    if (g_dtu_reconnect.state == DTU_RECON_IDLE) {
        if (++g_dtu_cmd.response_num > g_dtu_reconnect.offline_timeout_ticks) {
            g_dtu_cmd.net_status = USR_STATE_OFF;
            g_dtu_reconnect.state = DTU_RECON_OFFLINE;
            g_dtu_reconnect.attempt_count = 0;
            g_dtu_reconnect.current_delay_ticks = g_dtu_reconnect.initial_retry_delay_ticks;
            g_dtu_reconnect.next_attempt_tick = g_dtu_reconnect.tick_100ms + g_dtu_reconnect.current_delay_ticks;

            LOGT("warn: offline detected (>%ds no response), start reconnect\n",
                 g_dtu_reconnect.offline_timeout_ticks / 10);
        }
        return;
    }

    // 离线路径：根据状态机执行
    switch (g_dtu_reconnect.state) {
    case DTU_RECON_OFFLINE:
    case DTU_RECON_WAIT_DELAY:
        if (g_dtu_reconnect.tick_100ms >= g_dtu_reconnect.next_attempt_tick) {
            g_dtu_reconnect.state = DTU_RECON_WAIT_DELAY;
            dtu_reconnect_try_once();
            // 如果达到硬件重启阈值，执行硬重启
            if (g_dtu_reconnect.attempt_count >= g_dtu_reconnect.max_retries_before_hw_reboot) {
                LOGT("warn: reconnect attempts reach %d, HW reboot\n", g_dtu_reconnect.max_retries_before_hw_reboot);
                dtu_reconnect_hw_reboot();
            }
        }
        break;

    case DTU_RECON_REBOOT_COOLDOWN:
        if (g_dtu_reconnect.tick_100ms >= g_dtu_reconnect.next_attempt_tick) {
            LOGT("reconnect: cooldown finished, AT poweron\n");
            g_dtu_reconnect.state = DTU_RECON_WAIT_DELAY;
            g_dtu_reconnect.next_attempt_tick = g_dtu_reconnect.tick_100ms + g_dtu_reconnect.initial_retry_delay_ticks;
            APP_RTU_AT_Poweron();
        }
        break;

    default:
        break;
    }
}