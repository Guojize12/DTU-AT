#pragma once
#include "app_dtu.h"
#include "app_rtu_at_core.h"

/*
 * 统一的DTU断线重连策略模块
 * - 100ms tick驱动（由 APP_DTU_Callback() 每次调用）
 * - 可配置：离线判定时长、重试间隔（指数退避）、最大重试、硬件重启及冷却
 */

typedef enum {
    DTU_RECON_IDLE = 0,            // 在线或未进入重连
    DTU_RECON_OFFLINE,             // 已判定离线
    DTU_RECON_WAIT_DELAY,          // 等待下次重试窗口
    DTU_RECON_REBOOT_COOLDOWN,     // 硬件重启后冷却
} dtu_recon_state_t;

typedef struct {
    // 配置参数（单位均按100ms tick）
    uint16_t offline_timeout_ticks;       // 离线判定阈值（默认900=90秒）
    uint16_t initial_retry_delay_ticks;   // 初次重试延迟（默认30=3秒）
    uint16_t backoff_multiplier;          // 退避倍数（默认2）
    uint16_t max_retry_delay_ticks;       // 最大退避延迟（默认300=30秒）
    uint8_t  max_retries_before_hw_reboot;// 达到N次失败后触发硬件重启（默认3）
    uint8_t  max_total_hw_reboots;        // 单次会话最大硬件重启次数（默认3）
    uint16_t reboot_cooldown_ticks;       // 硬件重启冷却（默认600=60秒）
    uint8_t  target_channel;              // 需要维持的TCP业务通道（默认0）

    // 运行时状态
    dtu_recon_state_t state;
    uint32_t next_attempt_tick;
    uint16_t current_delay_ticks;
    uint8_t  attempt_count;
    uint8_t  hw_reboot_count;
    uint32_t tick_100ms;
} dtu_reconnect_t;

extern dtu_reconnect_t g_dtu_reconnect;

// 初始化与周期驱动
void DTU_Reconnect_Init(void);
void DTU_Reconnect_Tick(void);

// 事件通知
void DTU_Reconnect_OnAck(void);                 // 收到服务器任何有效应答（心跳、时间、上电、数据等）
void DTU_Reconnect_OnTcpDisconnect(uint8_t ch); // URC上报某通道断开