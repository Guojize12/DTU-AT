
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

// 全局变量声明，用于监控主循环健康状态
extern uint32_t g_main_loop_heartbeat;

void APP_MAIN_Handle(void);

#ifdef __cplusplus
}
#endif
#endif /*__APP_MAIN_H */

/*****END OF FILE****/
