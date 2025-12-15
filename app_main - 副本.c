#include "app_config.h"
#include "app_main.h"
#include "bsp_uart.h"
#include "app_user.h"
#include "app_memory.h"

// OTA测试模式：1=启用（上电自动OTA） 0=禁用
#define OTA_TEST_MODE  0

static Timer g_timer_iwdg = {0};
/**
  * @brief  喂看门狗
  * @param  None.
  * @retval None.
  */
static void APP_MAIN_Callback_Iwdg(void)
{
    BSP_IWDG_Refresh();
}

/**
  * @brief  回写固件
  */
void APP_MAIN_Write_Back(void)
{
    LOG("write back retry times[%d]\n",g_user_config.ota_config.write_back_times);
    EEPROM_FLASH_WriteU16(APP_IAP_ADDR_WRITE_BACK,++g_user_config.ota_config.write_back_times);//记录回写次数
    g_user_config.ota_config.firmware_size=APP_IAP_Read_App_Size();

    APP_IAP_Handle();
}
/**
* @brief  开机检测和OTA处理
  */
int APP_MAIN_Ota_Chk(void)
{
    int ret = USR_OTA_FALSE;
    uint32_t ota_flag = 0;
    
    APP_USER_Id_Get();
    
    LOGT("========================================\n");
    LOGT("BOOTLOADER: OTA Check Starting\n");
    LOGT("eeprom_enable = %d (expect: %d)\n", g_user_config.sys_config.eeprom_enable, EEPROM_ENABLE_VALUE);
    LOGT("========================================\n");
    
    // 如果EEPROM未初始化(新设备),先初始化它
    if(g_user_config.sys_config.eeprom_enable != EEPROM_ENABLE_VALUE)
    {
        LOGT("Device not initialized, initializing EEPROM...\n");
        EEPROM_FLASH_WriteU16(CFG_ADDR_EN, EEPROM_ENABLE_VALUE); // 写入使能标志
        EEPROM_FLASH_WriteU32(CFG_ADDR_DID, DID_DEFAULT); // 写入默认设备ID
        g_user_config.sys_config.eeprom_enable = EEPROM_ENABLE_VALUE;
        g_user_config.sys_config.device_id = DID_DEFAULT;
        LOGT("EEPROM initialized, device_id=%d\n", DID_DEFAULT);
    }
	
    if(EEPROM_ENABLE_VALUE == g_user_config.sys_config.eeprom_enable)
    {
        // 读取OTA标志
        ota_flag = EEPROM_FLASH_ReadU32(APP_IAP_ADDR_STATUS_OTA);
        LOGT("OTA flag read: %d (magic=%d)\n", ota_flag, OTA_FLAG_MAGIC_NUMBER);
        
        if(ota_flag == OTA_FLAG_MAGIC_NUMBER) // APP触发的OTA或测试模式
        {
            LOGT("ota:app triggered OTA, enter ota mode...\n");
            g_user_config.ota_config.ota_disenable = 0;  // 激活OTA模式（允许OTA状态机执行）
            // OTA模式，在主循环中处理（不在这里初始化）
            ret = USR_OTA_TRUE;  // 标记为OTA模式
        }
        else
        {
            // 正常的IAP检查流程
            ret = APP_IAP_Check();
        }
    }
    
    if(ret == USR_OTA_FALSE)//没有ota
    {
        LOGT("No OTA, jumping to APP at 0x%08X...\n", ApplicationAddress);
        ret = APP_IAP_App_Run();//跳转APP
    }

    if(ret >= OTA_ERR_APP_MD5) //记录错误
    {
        APP_LOG_Write(ret);
    }
    return ret;
}



static void APP_MAIN_Tmr_Init(void)
{
    BSP_TIMER_Init(&g_timer_iwdg, APP_MAIN_Callback_Iwdg, TIMEOUT_2S, TIMEOUT_2S);
    BSP_TIMER_Start(&g_timer_iwdg);
}

/**
  * @brief  功能初始化
  * @param  None.
  * @retval None.
  */
void APP_MAIN_Init(void)
{
    APP_MAIN_Tmr_Init();
    /* add app init*/
    APP_CONFIG_Init();
    APP_DTU_Init();//建立1  APP_RTU_AT_Config_Handle 建立2 APP_RTU_AT_Config_Handle_Err  
    /* add app init end */
    APP_VERSION_Print();
    APP_USER_Init();//建立1 _Loop 
//  APP_RTU_AT_Init();
}
/**
  * @brief  主循环
  * @param  None.
  * @retval None.
  */
// 全局变量用于监控主循环健康状态
void APP_MAIN_Handle(void)
{
//	APP_IAP_App_Run();//跳转APP
	
    int ret = APP_MAIN_Ota_Chk();

    // 如果检测到错误日志（USR_LOG_SEND），设置为待机模式，防止误进入OTA
    if(ret == USR_LOG_SEND)
    {
        g_user_config.ota_config.ota_disenable = 1;
        LOGT("Error log detected (ret=%d), entering standby mode (no OTA)\n", ret);
    }

    APP_MAIN_Init();  // 初始化

    if(ret == USR_FIRMWARE_BACKUP_OK)//有备份
    {
        APP_MAIN_Write_Back();
    }
    
    if(ret != USR_OTA_TRUE)  // 非OTA模式
    {
        EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA,0);//升级标志复位（改为U32与读取匹配）
    }
    else
    {
        LOGT("ota:enter OTA mode, starting main loop... (disenable=%d)\n", g_user_config.ota_config.ota_disenable);
    }
    

    BSP_IWDG_Refresh();
    while (1)
    {
        /* add bsp handle*/
        /** UART*/
        BSP_UART_Handle();
        /** TIMER*/
        BSP_TIMER_Handle();
        /* add bsp handle end */

        /* add app handle*/
        APP_DTU_Handle();  // 包含 APP_DTU_Rec_Handle() 和 APP_DTU_Ota_Handle()
        /* add app handle end */

        /* add log handle*/
        /** RTT*/
        USER_LOG_Input_Handle();
        /* add log end */
       HAL_Delay(1);
    }
}

/*****END OF FILE****/



