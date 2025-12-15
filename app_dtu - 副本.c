#include "app_dtu.h"
#include "app_config.h"
#include "app_iap.h"
    
#define APP_DTU_UART          BSP_UART3
#define APP_DTU_UART_BUF      g_uart_buf[APP_DTU_UART]

#define APP_DTU_SIGNAL_TIMEOUT  (1200)  //dtu连接超时判断时间:2分钟

static app_rtu_rx_def g_dtu_rx; //dtu 多通道接收数据
// OTA相关全局变量
MD5_CTX g_cont = {0};
extern uint16_t g_progress;

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
    .normal_interval = 30,
    .work_interval = 100,
};

void APP_DTU_Status_Reset(void)
{
    g_dtu_cmd.cnt_status = USR_ERROR;
    g_dtu_cmd.power_on_status = USR_ERROR;
    g_dtu_cmd.response_cmd = DTU_CNT_STEP0;
    g_dtu_cmd.send_num = 0;
    g_dtu_cmd.time_num = 0; //dtu上传时间判断计时清0

    g_dtu_cmd.response_num = APP_DTU_SIGNAL_TIMEOUT;//dtu连接状态判断条件为 未连接

    g_dtu_cmd.gps_send_interval = 31; //定位上传间隔31s
    g_dtu_cmd.gps_enable = 0; //不支持定位功能
    
    // OTA相关状态初始化
    g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP0;
    g_dtu_cmd.send_times = 0;
    g_dtu_cmd.time_status = USR_FALSE;
    g_dtu_cmd.response_status = USR_FALSE;
}

//dtu串口直发
void APP_DTU_Send_Buf(uint8_t *buffer, uint16_t size)
{
    BSP_UART_Transmit(APP_DTU_UART, buffer, size);
//  LOG("%s\n",buffer);
}

//dtu多通道发
void APP_DTU_Send(uint8_t *buffer, uint16_t size)
{

    APP_RTU_AT_Tx_Chl(0, buffer, size);
}
//指令头打包
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

    g_dtu_cmd.cmd_last = cmd;
}

//设备通用应答
void APP_DTU_Response_Result(uint16_t cmd, uint8_t   state, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 1, cmd, rxBuf[20]);
    txBuf[23] = state;
    uint16_t crc16 = bsp_crc16(txBuf + 23, 1);
    txBuf[24] = (uint8_t)crc16 & 0xFF;
    txBuf[25] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, 26);
}

//设备心跳
void APP_DTU_Send_Hearbeat(void)
{
    LOGT("smt:ht\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_HEARTBEAT, 0);
    APP_DTU_Send(txBuf, 23);

//  LOGT("send heart_beat to remote [%d]\n", 23);
//  LOG_HEX(txBuf, 23);
    if (g_dtu_cmd.net_status == USR_STATE_OFF)
    {
        LOGT("smt:ht,no net to cnt..\n");
    }
}

//设备主动获取服务器时间
void APP_DTU_GetServerTime(void)
{
    LOGT("smt:get time\n");
    uint8_t txBuf[32] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_TIMESYNC, 0);
    APP_DTU_Send(txBuf, 23);
}

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
    APP_VERSION_Get_Soft(version);//软件版本
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

    APP_VERSION_Get_Hard(version);//硬件版本
    txBuf[num++] = version[0];
    txBuf[num++] = version[1];
    txBuf[num++] = version[2];

//    APP_CONFIG_Get_Model();
    memcpy(txBuf + num, g_app_cfg.model, 20);
    num += 20;

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_POWE_ON_STATUS, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}


//sim卡信息
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

    txBuf[num++] = 20; //Sim卡号ICCID长度
    memcpy(txBuf + num, g_app_rtu_sim.iccid, 20);
    num += 20;
    txBuf[num++] = 0; //Sim入网号长度
    txBuf[num++] = g_app_rtu_sim.signal_per; //信号强度

    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_DEVICE_SIM, 0);

    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);

}

// ========== OTA相关函数 ==========

// 0. OTA触发设置（用于APP程序）
void APP_DTU_Ota_Set(uint8_t opt)
{
    // 清除重启次数、异常LOG、回写次数
    EEPROM_FLASH_WriteU16(APP_IAP_ADDR_RESET, 0);
    APP_LOG_Write(0);
    EEPROM_FLASH_WriteU16(APP_IAP_ADDR_WRITE_BACK, 0);
    
    if (opt > 0)
    {
        EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, OTA_FLAG_MAGIC_NUMBER); // 设置升级标志
    }
    else
    {
        EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, 0); // 清除升级标志（改为U32与读取匹配）
    }
    
    BSP_CONFIG_System_Reset(); // 系统复位
}

// BOOT复位函数（清除所有状态并重启，用于调试）
void APP_DTU_Boot_Reset(void)
{
    EEPROM_FLASH_WriteU16(APP_IAP_ADDR_RESET, 0);       // 复位重启次数
    APP_LOG_Write(0);                                    // 清除异常LOG
    EEPROM_FLASH_WriteU16(APP_IAP_ADDR_WRITE_BACK, 0); // 清除回写次数
    BSP_CONFIG_System_Reset();                           // 系统复位
}

// 1. 获取远程固件信息
void APP_DTU_Send_Get_Firmware_Msg(void)
{
    uint8_t txBuf[64] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 0, DTU_CMD_DEVICE_OTA_MSG, 0);
    APP_DTU_Send(txBuf, 23);
    if(BSP_CONFIG_Show_Get() == 250)
    {
        LOGT("ota:send firmware info\n");
        LOG_HEX(txBuf, 23);
    }
    LOGT("ota:request firmware info\n");
}

// 2. 固件信息解析
void APP_DTU_Ota_Msg_Set(uint8_t *rxBuf, uint16_t rxLen)
{
    IEEE754 ie_data;
    
    g_user_config.ota_config.ota_status = rxBuf[23];
    LOGT("ota:status=%d\n", g_user_config.ota_config.ota_status);
    
    ie_data.u8_buf[3] = rxBuf[24];
    ie_data.u8_buf[2] = rxBuf[25];
    ie_data.u8_buf[1] = rxBuf[26];
    ie_data.u8_buf[0] = rxBuf[27];
    g_user_config.ota_config.firmware_size = ie_data.u32;
    
    if (g_user_config.ota_config.firmware_size > (APP_IAP_ADDR_APP_END - APP_IAP_ADDR_APP_START))
    {
        g_user_config.ota_config.ota_status = 3;
        LOGT("err:firmware size[%d]B > app flash size[%d]B\n",
             g_user_config.ota_config.firmware_size,
             (APP_IAP_ADDR_APP_END - APP_IAP_ADDR_APP_START));
        return;
    }
    
    memcpy(g_user_config.ota_config.firmware_md5, rxBuf + 28, 16);
    
    LOGT("ota:firmware size=%d\n", g_user_config.ota_config.firmware_size);
    LOGT("ota:md5=");
    LOG_HEX(g_user_config.ota_config.firmware_md5, 16);
}

// 3. 准备固件备份区
void APP_DTU_Firmware_Backup_Ready(void)
{
    // 备份区首地址
    g_user_config.ota_config.firmware_addr_write = APP_IAP_ADDR_BACKUP_ADDR;
    
    // 准备备份区 - 擦除Flash
    LOGT("ota:erase backup flash [0x%08X - 0x%08X]\n",
         g_user_config.ota_config.firmware_addr_write,
         g_user_config.ota_config.firmware_addr_write + g_user_config.ota_config.firmware_size);
    
    BSP_FLASH_Erase_Pages(g_user_config.ota_config.firmware_addr_write,
                          g_user_config.ota_config.firmware_addr_write + g_user_config.ota_config.firmware_size);
    
    // MD5初始化
    MD5_Init(&g_cont);
    
    // 新固件首地址
    g_user_config.ota_config.firmware_addr_read = 0;
    
    // 分包长度
    g_user_config.ota_config.firmware_size_sub = FIRMWARE_SUB_LEN;
    
    LOGT("ota:backup area ready\n");
}

// 4. 请求固件分包
void APP_DTU_Send_Get_Firmware_Sub(uint32_t addr, uint32_t len)
{
    uint8_t txBuf[64] = {0};
    IEEE754 ie_data;
    
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, 8, DTU_CMD_DEVICE_OTA_FIRMWARE, 0);
    
    // 打包地址
    ie_data.u32 = addr;
    txBuf[23] = ie_data.u8_buf[3];
    txBuf[24] = ie_data.u8_buf[2];
    txBuf[25] = ie_data.u8_buf[1];
    txBuf[26] = ie_data.u8_buf[0];
    
    // 打包长度
    ie_data.u32 = len;
    txBuf[27] = ie_data.u8_buf[3];
    txBuf[28] = ie_data.u8_buf[2];
    txBuf[29] = ie_data.u8_buf[1];
    txBuf[30] = ie_data.u8_buf[0];
    
    uint16_t crc16 = bsp_crc16(txBuf + 23, 8);
    txBuf[31] = (uint8_t)crc16 & 0xFF;
    txBuf[32] = (uint8_t)(crc16 >> 8);
    
    APP_DTU_Send(txBuf, 33);
}

// 5. 固件头检查
uint8_t APP_DTU_Firmware_Head_Check(uint8_t *rxBuf)
{
    uint8_t ret = 0;
    
    if (memcmp(rxBuf + 32, g_app_cfg.model, strlen(g_app_cfg.model)) == 0)
    {
        ret = 1;
        LOGT("ota:model check ok\n");
    }
    else
    {
        char buf[20] = {0};
        memcpy(buf, rxBuf + 32, 20);
        LOGT("err:model mismatch, new\"%s\", old\"%s\"\n", buf, g_app_cfg.model);
    }
    
    return ret;
}

// 6. 固件分包备份
void APP_DTU_Firmware_Backup(uint8_t *rxBuf, uint16_t rxLen)
{
    g_user_config.ota_config.ota_status = rxBuf[23];
    
    if (g_user_config.ota_config.ota_status == 1)
    {
        IEEE754 ie_data;
        uint32_t addr, len;
        
        ie_data.u8_buf[3] = rxBuf[24];
        ie_data.u8_buf[2] = rxBuf[25];
        ie_data.u8_buf[1] = rxBuf[26];
        ie_data.u8_buf[0] = rxBuf[27];
        addr = ie_data.u32;
        
        ie_data.u8_buf[3] = rxBuf[28];
        ie_data.u8_buf[2] = rxBuf[29];
        ie_data.u8_buf[1] = rxBuf[30];
        ie_data.u8_buf[0] = rxBuf[31];
        len = ie_data.u32;
        
        if (g_user_config.ota_config.firmware_size_sub == len &&
            g_user_config.ota_config.firmware_addr_read == addr)
        {
            // 固件头检测
            if (g_user_config.ota_config.firmware_addr_read == 0)
            {
                if (APP_DTU_Firmware_Head_Check(rxBuf) == 0)
                {
                    g_user_config.ota_config.ota_status = 2;
                    return;
                }
            }
            
            // 写入固件数据
            uint8_t pBuf[FIRMWARE_SUB_LEN];
            memcpy(pBuf, rxBuf + 32, len);
            BSP_FLASH_Write(g_user_config.ota_config.firmware_addr_write, (uint32_t *)pBuf, len);
            
            // 更新MD5
            MD5_Update(&g_cont, rxBuf + 32, len);
            
            // 更新地址
            g_user_config.ota_config.firmware_addr_write += len;
        }
        else
        {
            g_user_config.ota_config.ota_status = 0;
            LOGT("err:packet mismatch, expect addr=%d,len=%d, got addr=%d,len=%d\n",
                 g_user_config.ota_config.firmware_addr_read,
                 g_user_config.ota_config.firmware_size_sub,
                 addr, len);
        }
    }
}

// 7. 固件分包更新
int APP_DTU_Send_Get_Firmware_Msg_Sub(void)
{
    // 下一包地址
    g_user_config.ota_config.firmware_addr_read += g_user_config.ota_config.firmware_size_sub;
    g_progress = (g_user_config.ota_config.firmware_addr_read * 100 / g_user_config.ota_config.firmware_size);
    LOGT("ota:download -> %d%%\n", g_progress);
    
    // 固件获取完成
    if (g_user_config.ota_config.firmware_addr_read >= g_user_config.ota_config.firmware_size)
    {
        unsigned char digest[16];
        MD5_Final(&g_cont, digest);
        
        if (memcmp(digest, g_user_config.ota_config.firmware_md5, 16) == 0)
        {
            EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, 2); // 记录固件备份成功
            APP_IAP_Write_App_Size(g_user_config.ota_config.firmware_size); // 记录固件大小
            APP_IAP_Write_App_Md5(g_user_config.ota_config.firmware_md5, 16); // 记录固件MD5
            APP_LOG_Write(0);  // 清除错误日志
            LOGT("ota:md5 check ok, firmware download complete!\n");
            return USR_TRUE; // 与HC32一致
        }
        else
        {
            LOGT("err:md5 check failed!\n");
            LOGT("md5 expected: ");
            LOG_HEX(g_user_config.ota_config.firmware_md5, 16);
            LOGT("md5 received: ");
            LOG_HEX(digest, 16);
            return USR_APP_MD5_ERR; // 与HC32一致
        }
    }
    
    // 继续请求下一包
     //下一包长度
     if((g_user_config.ota_config.firmware_size-g_user_config.ota_config.firmware_addr_read)>FIRMWARE_SUB_LEN)
     {
         g_user_config.ota_config.firmware_size_sub = FIRMWARE_SUB_LEN;
     }
     else
     {
         g_user_config.ota_config.firmware_size_sub = g_user_config.ota_config.firmware_size-g_user_config.ota_config.firmware_addr_read;
     }
 #if 0
     LOGT("next read addr[%d],len[%d]\n",g_user_config.ota_config.firmware_addr_read,g_user_config.ota_config.firmware_size_sub);
 #endif
 
     return USR_FALSE;
}

// 8. 获取远程固件（包装函数）
void APP_DTU_Send_Get_Firmware(void)
{
    APP_DTU_Send_Get_Firmware_Sub(g_user_config.ota_config.firmware_addr_read,
                                   g_user_config.ota_config.firmware_size_sub);
}

// 9. OTA连接和状态机处理（参考HC32完整版本）
// 注意：原项目中已有APP_DTU_Connect_Remote_Handle用于开机联网，此处重命名为OTA专用
uint8_t APP_DTU_Ota_Connect_Handle(void)
{
    uint8_t result = USR_FALSE;
    
   
    switch(g_dtu_cmd.cmd_last)
    {
    case DTU_CMD_DEVICE_TIMESYNC:
        if(g_dtu_cmd.time_status == USR_TRUE) //获取时间成功
        {
            LOGT("ota:time sync OK, step: %d -> %d\n", g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP1);
            
            // 时间同步成功，说明网络已恢复，清除旧的错误日志（允许重新尝试OTA）
            uint16_t old_log = APP_LOG_Read();
            if(old_log >= OTA_ERR_APP_MD5)
            {
                LOGT("ota:clear old error log [%d], network recovered\n", old_log);
                APP_LOG_Write(0);
            }
            
            g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP1;
            g_dtu_cmd.send_times = 0;
        }
        else if(g_dtu_cmd.response_status == USR_TIMEROUT)
        {
            if(++g_dtu_cmd.send_times > 20)  // 添加20次限制
            {
                LOGT("ota:EXIT OTA MODE - time sync timeout, exceed 20 retries (times=%d)\n", g_dtu_cmd.send_times);
                APP_LOG_Write(OTA_ERR_TIME_SYNC_TIMEOUT);  // 记录错误
                result = USR_OTA_FALSE;  // 退出OTA
            }
            else
            {
                LOGT("ota:time sync timeout, step: %d -> %d [%d/20]\n", 
                     g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP0, g_dtu_cmd.send_times);
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP0;
                LOG("get remote time retry [%d]\n", g_dtu_cmd.send_times);
            }
        }
        break;
        
    case DTU_CMD_DEVICE_POWE_ON_STATUS:
        if(g_dtu_cmd.response_status == USR_TRUE)//开机状态发送成功
        {
            if(APP_LOG_Read() >= OTA_ERR_APP_MD5) //上次升级失败
            {
                g_user_config.ota_config.ota_disenable = 1;
                LOGT("ota:EXIT OTA MODE - power on status sent, last ota failed (log=%d)\n", APP_LOG_Read());
            }
            else
            {
                LOGT("ota:power on OK, step: %d -> %d\n", g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP2);
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP2;
                LOGT("ota:power on status sent\n");
                g_dtu_cmd.send_times = 0;
            }
        }
        else if(g_dtu_cmd.response_status == USR_TIMEROUT)
        {
            if(++g_dtu_cmd.send_times > 20)  // 添加20次限制
            {
                LOGT("ota:power on timeout, exceed 20 retries, exit OTA\n");
//                APP_LOG_Write(OTA_ERR_POWER_ON_TIMEOUT);  // 记录错误
                result = USR_OTA_FALSE;  // 退出OTA
            }
            else
            {
                LOGT("ota:power on timeout, step: %d -> %d [%d/20]\n", 
                     g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP1, g_dtu_cmd.send_times);
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP1;
                LOG("send power on status retry [%d]\n", g_dtu_cmd.send_times);
            }
        }
        break;
        
    case DTU_CMD_DEVICE_OTA_MSG:
        if(g_dtu_cmd.response_status == USR_TRUE)//获取远程升级文件的信息 成功
        {
            if(g_user_config.ota_config.ota_status == 1)//文件存在可以升级
            {
                APP_DTU_Firmware_Backup_Ready();
          //     LOGT("ota:firmware info OK, step: %d -> %d\n", g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP3);
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP3;
            }
            else if(g_user_config.ota_config.ota_status == 3)//固件太长
            {
                LOGT("ota:EXIT OTA MODE - firmware too long\n");
                APP_LOG_Write(OTA_ERR_FIRMWARE_TOO_LONG); //记录 固件太长
                result = USR_OTA_FALSE;
            }
            else
            {
                LOGT("ota:EXIT OTA MODE - no upgrade available\n ");
                APP_LOG_Write(OTA_ERR_NO_OTA);//记录不可升级
                result = USR_OTA_FALSE;
            }
            g_dtu_cmd.send_times = 0;
        }
        else if(g_dtu_cmd.response_status == USR_TIMEROUT)
        {
            if(++g_dtu_cmd.send_times > 20)
            {
                LOGT("ota:EXIT OTA MODE - get firmware msg timeout\n");
                APP_LOG_Write(OTA_ERR_MSG_TIMEOUT);//记录固件信息超时错误
                result = USR_OTA_FALSE;
            }
            else
            {
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP2;
            }
            LOG("get firmware msg retry [%d]\n", g_dtu_cmd.send_times);
        }
        break;
        
    case DTU_CMD_DEVICE_OTA_FIRMWARE:
        if(g_dtu_cmd.response_status == USR_TRUE) //获取远程升级文件 成功
        {
            if(g_user_config.ota_config.ota_status == 1)//分包接受成功
            {
                int ret = APP_DTU_Send_Get_Firmware_Msg_Sub();
                if(ret == USR_TRUE)//固件下载成功
                {
                    result = USR_TRUE;
                }
                else if(ret == USR_APP_MD5_ERR)//固件MD5校验失败
                {
                    LOGT("ota:EXIT OTA MODE - firmware MD5 check failed\n");
                    APP_LOG_Write(OTA_ERR_FIRMWARE_MD5); //记录 下载固件MD5 校验错误
                    result = USR_OTA_FALSE;
                }
                else
                {
                 //   LOGT("ota:firmware packet OK, continue step: %d -> %d (ret=%d)\n", 
                //          g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP3, ret);
                    g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP3;
                }
                g_dtu_cmd.send_times = 0;
            }
            else if(g_user_config.ota_config.ota_status == 2)//固件头错误
            {
                LOGT("ota:EXIT OTA MODE - firmware head check failed\n");
                APP_LOG_Write(OTA_ERR_FIRMWARE_HEAD); //记录 下载固件头 校验错误
                result = USR_OTA_FALSE;
            }
            else
            {
                if(++g_dtu_cmd.send_times > 20)
                {
                    LOGT("ota:EXIT OTA MODE - firmware packet error, exceed 20 retries\n");
                    APP_LOG_Write(OTA_ERR_FIRMWARE_NUM);//记录获取分包升级包错误
                    result = USR_OTA_FALSE;
                }
                else
                {
                    LOGT("ota:firmware packet error (status=%d), retry step: %d -> %d [%d/20]\n",
                         g_user_config.ota_config.ota_status, g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP3, g_dtu_cmd.send_times);
                    g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP3;
                }
                LOG("get firmware sub err retry[%d]\n", g_dtu_cmd.send_times);
            }
        }
        if(g_dtu_cmd.response_status == USR_TIMEROUT) //获取远程升级文件 超时
        {
            if(++g_dtu_cmd.send_times > 20) //连续超时20次,升级失败
            {
                LOGT("ota:EXIT OTA MODE - firmware packet timeout, exceed 20 retries\n");
                APP_LOG_Write(OTA_ERR_FIRMWARE_TIMEOUT);//记录获取分包升级包超时错误
                result = USR_OTA_FALSE;
            }
            else
            {
                LOGT("ota:firmware packet timeout, retry step: %d -> %d [%d/20]\n",
                     g_dtu_cmd.dtu_step, POWER_ON_DTU_STEP3, g_dtu_cmd.send_times);
                g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP3;
            }
            LOG("get firmware sub timeout retry[%d]\n", g_dtu_cmd.send_times);
        }
        break;
    }
    
    switch(g_dtu_cmd.dtu_step)
    {
    case POWER_ON_DTU_STEP0:
        LOGT("STEP0 - get server time\n");
        g_dtu_cmd.response_status = USR_FALSE; // 【必需】发送前重置，等待新响应
        APP_DTU_GetServerTime();//获取服务器时间
        LOG("power on get time\n");
        break;
    case POWER_ON_DTU_STEP1:
        LOGT("STEP1 - send power on status\n");
        g_dtu_cmd.response_status = USR_FALSE; // 【必需】发送前重置，等待新响应
        APP_DTU_SendDTUPowerOnData();//开机状态
        LOG("power on status\n");
        break;
    case POWER_ON_DTU_STEP2:
        LOGT("STEP2 - get firmware info\n");
        g_dtu_cmd.response_status = USR_FALSE; // 【必需】发送前重置，等待新响应
        APP_DTU_Send_Get_Firmware_Msg();//获取远程升级文件的信息
        LOG("get firmware msg\n");
        break;
    case POWER_ON_DTU_STEP3:
        LOGT("STEP3 - get firmware packet (addr=%d, len=%d)\n",
             g_user_config.ota_config.firmware_addr_read,
             g_user_config.ota_config.firmware_size_sub);
        g_dtu_cmd.response_status = USR_FALSE; // 【必需】发送前重置，等待新响应
        APP_DTU_Send_Get_Firmware();//获取远程升级文件
        break;
    default:
    //    LOGT("DEFAULT (step=%d)\n", g_dtu_cmd.dtu_step);
        break;
    }
    
    //LOGT("after exec, step: %d -> MAX\n", g_dtu_cmd.dtu_step);
    g_dtu_cmd.dtu_step = POWER_ON_DTU_STEP_MAX;
    
    return result;
}

// 10. OTA主处理函数（参考HC32完整版本）
void APP_DTU_Ota_Handle(void)
{
    static uint32_t ota_timeout_counter = 0; // 超时计数器
    static uint32_t loop_counter = 0;        // 统一计数器（用于心跳和实时数据）
    static uint32_t last_ota_exec_time = 0;  // OTA状态机上次执行时间

    // OTA状态机频率控制：限制执行频率，适配4G网络延迟
    // 避免状态机执行过快导致请求堆积和响应乱序
    if(g_user_config.ota_config.ota_disenable == 0)  // 只在OTA模式下限制频率
    {
        uint32_t current_time = BSP_TIMER_Ticks_Get();
        if(current_time - last_ota_exec_time < 200)  // 最小200ms间隔
        {
            return;  // 距离上次执行不足100ms，跳过本次执行
        }
        last_ota_exec_time = current_time;
    }

    // 每100次调用打印一次，减少日志量
    static uint32_t ota_handle_counter = 0; // 调用计数器（用于减少日志频率）
    if((++ota_handle_counter % 3000) == 0)  // 每1500次200ms    打印一次状态，即30秒打印一次
    {
        LOG("ota:handle [%d], disenable=%d, step=%d, resp_sta=%d, tcp_cnt=%d, poweron=%d\n",
             ota_handle_counter, 
             g_user_config.ota_config.ota_disenable,
             g_dtu_cmd.dtu_step,
             g_dtu_cmd.response_status,
             g_app_rtu_at.tcp_cnt[0],
             g_app_rtu_at.poweron);
    }
    
    // 统一计数器自增（假设主循环1ms）
    loop_counter++;
    
    // 判断是否在OTA过程中：只要 ota_disenable==0，就表示正在OTA，不应发送心跳/实时数据
    // 注意：dtu_step在执行完每个步骤后会被设置为STEP_MAX，所以不能用step来判断
    uint8_t is_ota_mode = (g_user_config.ota_config.ota_disenable == 0);
    
    // 心跳发送：每10秒一次（OTA模式下不发送，避免干扰）
    if(loop_counter % 10000 == 0)  // 10秒 = 10000ms
    {
        // 只在TCP连接正常且非OTA模式时发送
        if(g_app_rtu_at.poweron == 2 && g_app_rtu_at.tcp_cnt[0] > 0 && !is_ota_mode)
        {
            APP_DTU_Send_Hearbeat();
            LOGT("boot:send heartbeat\n");
        }
    }
    
    // 实时数据发送：每30秒一次（OTA模式下不发送，避免干扰）
    if(loop_counter % 30000 == 0)  // 30秒 = 30000ms
    {
        // 只在TCP连接正常、待机模式且非OTA模式时发送
        if(g_app_rtu_at.poweron == 2 && g_app_rtu_at.tcp_cnt[0] > 0 && 
           g_user_config.ota_config.ota_disenable > 0 && !is_ota_mode)
        {
            APP_DTU_Send_DTURealTimeRecord();
            LOGT("boot:send realtime data\n");
        }
    }
    
    // 防止计数器溢出：每60秒清零一次
    if(loop_counter >= 60000)  // 60秒 = 60000ms
    {
        loop_counter = 0;
    }
    
    if(g_user_config.ota_config.ota_disenable > 0)//待机模式
    {
        if(BSP_TIMER_Ticks_Get() % (1*60*1000) == 0)
        {
            g_dtu_cmd.time_status = 0;
            APP_DTU_GetServerTime();
            LOGT("bootloader standby...\n");
        }
    }
    else
    {
       // 简单的超时检测：如果response_status长时间保持USR_FALSE，转换为USR_TIMEROUT
        if(g_dtu_cmd.response_status == USR_FALSE)//
        {
            ota_timeout_counter++;
            if(ota_timeout_counter > 50) // 约5秒超时（假设主循环200ms一次，50次=10000ms=10秒）
            {
                g_dtu_cmd.response_status = USR_TIMEROUT;
                ota_timeout_counter = 0;
                LOG("ota:response timeout, set to TIMEROUT (counter=%d)\n", ota_timeout_counter);
            }
        }
        else
        {
            ota_timeout_counter = 0; // 收到响应，清零计数器
        }
        
        uint8_t ota_result = APP_DTU_Ota_Connect_Handle();
        if(ota_result != USR_FALSE)//USR_FALSE =0，USR_OTA_FALSE=3
        {
            LOG("ota:Connect_Handle returned %d (USR_OTA_FALSE=%d)\n", ota_result, USR_OTA_FALSE);
            
            if(APP_LOG_Read() >= OTA_ERR_APP_MD5)
            {
                LOGT("Not Ota, to reset!! (log=%d)\n", APP_LOG_Read());
                APP_LOG_Write(0);  // 清除错误日志，防止下次启动再次触发
                EEPROM_FLASH_WriteU32(APP_IAP_ADDR_STATUS_OTA, 0); // 清除OTA标志（改为U32与读取匹配）
                g_user_config.ota_config.ota_disenable = 1;  // 设置待机模式，防止重启后再次进入OTA
                HAL_Delay(100);
                BSP_CONFIG_System_Reset();
            }
            
            LOG("ready to write back..\n");
            APP_IAP_Handle();
        }
    }
}
//时间同步
void APP_DTU_TimeSync_Set(uint8_t *rxBuf, uint16_t rxLen)
{

    LOGT("rtc %d-%d\n", rxBuf[23], rxBuf[24]);
    uint16_t ttt = (rxBuf[23] * 256) + rxBuf[24];
    
    // 修复：判断是否为2位年份（服务器下发校时）
    if (ttt < 100)  // 服务器下发的2位年份
    {
        ttt += 2000;  // 补上2000
    }
    
    g_bsp_rtc.year = ttt % 100;
    g_bsp_rtc.month = rxBuf[25];
    g_bsp_rtc.day = rxBuf[26];
    g_bsp_rtc.hour = rxBuf[27];
    g_bsp_rtc.minute = rxBuf[28];
    g_bsp_rtc.second = rxBuf[29];
    if (ttt > 2019)
    {
        BSP_RTC_Set(g_bsp_rtc);
    }
    LOGT("timing:%04d-%02d-%02d %02d:%02d:%02d\n", 
         (ttt > 100 ? ttt : 2000 + g_bsp_rtc.year), 
         g_bsp_rtc.month, g_bsp_rtc.day,
         g_bsp_rtc.hour, g_bsp_rtc.minute, g_bsp_rtc.second);

}

//设备心跳
void APP_DTU_Response_Hearbeat(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    uint8_t txBuf[64] = {0};
    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, 0, cmd, rxBuf[20]);
    APP_DTU_Send(txBuf, 23);
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
    //TODO 存储 实时数据 上传间隔
//    APP_CONFIG_User_Upload_Time_Set(g_dtu_remote_cmd.normal_interval,g_dtu_remote_cmd.work_interval);
    APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
    LOGT("updata interval normal[%d],work[%d]\n", g_dtu_remote_cmd.normal_interval, g_dtu_remote_cmd.work_interval);

}
/* 参数设置 */
void nucRecDTUSetParaData(uint16_t cmd, uint32_t  addr, uint32_t dat, uint8_t *rxBuf, uint16_t rxLen)
{
    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;
    extern uint16_t flash_save_enable;
    extern uint16_t alarm_button_or_dwin;

    uint8_t ret = DTU_CMD_RESPONSE_SUCCESS;
    
    
    LOGT("set addr:%d --value:%d\n", addr, dat);
    switch (addr)
    {
    // 公共参数
    case 0x0001://修改设备号
        APP_CONFIG_Did_Set(dat);
        break;
    case 0x0002://正常上传间隔  秒
        g_dtu_remote_cmd.normal_interval = dat * 10; // 秒转100ms单位：秒×10
        //TODO 存储到EEPROM: APP_CONFIG_User_Upload_Time_Set(dat*10,0);
        break;
    case 0x0003://运行时上传间隔 秒
        g_dtu_remote_cmd.work_interval = dat * 10; // 秒转100ms单位：秒×10
        //TODO 存储到EEPROM: APP_CONFIG_User_Upload_Time_Set(0,dat*10);
        break;
    case 0x0005://锁机 参数 0正常， 1锁机
        //TODO 实现锁机功能
        break;
    case 0x0006://迪文电表支持 不为零时上传参数消息
        if (dat != 0) {
        }
        return; // 不发送通用回复
    case 0x0007://设备登录密码 6位数字
        //TODO APP_CONFIG_Set_DwinPassword(dat);
        break;
    case 0x0008://心跳间隔 秒
        //TODO 实现心跳间隔设置
        break;
    case 0x0009://自动校准配置 0关闭 1打开
        //TODO 实现自动校准配置
        break;
    case 0x000A://reboot重启
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("system:rebooting...\n");
        BSP_DELAY_MS(100);
        BSP_CONFIG_System_Reset();
        return;
    case 0x000B://DTU重启
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("dtu:rebooting...\n");
        BSP_DELAY_MS(200);
        //TODO BSP_POWER_Reboot(0); // 如果硬件支持
        return;
    case 0x000C://恢复出厂
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("factory:resetting...\n");
        BSP_DELAY_MS(200);
        //TODO BSP_EEPROM_Reset_Factory();
        return;
    case 0x000D://静音 0正常 1静音断电保存 2静音断电不保存
        //TODO 实现静音功能
        break;
    case 0x000E://运行模式 0正常 1调试模式 2调试10分钟 3非工作模式
        //TODO 实现运行模式切换
        break;
    case 0x000F://设备控制台透传 0关闭 1开启5分钟 2永久开启
        //TODO 实现透传功能
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
/* 参数设置 */

/* 参数获取 0x8014*/
void APP_DTU_Cmd_Config_Get(uint16_t cmd, uint32_t addr, uint8_t pid)
{
    IEEE754 ie_data = {0};

    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    ie_data.u32 = addr;//参数ID
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    txBuf[num++] = 1; //参数个数

    extern float MEAN_DEVIATION_THRESHOLD;
    extern float SENSOR_DEVIATION_THRESHOLD;
    extern float VARIANCE_THRESHOLD;
    extern float TREND_THRESHOLD;
    extern float DEFECT_SCORE_THRESHOLD;
    
    switch (addr)
    {
    // 公共参数
    case 0x0001://设备号
        ie_data.u32 = g_app_cfg.did;
        break;
    case 0x0002://正常上传间隔  秒
        ie_data.u32 = g_dtu_remote_cmd.normal_interval / 10;
        break;
    case 0x0003://运行时上传间隔 秒
        ie_data.u32 = g_dtu_remote_cmd.work_interval / 10;
        break;
    case 0x0005://锁机 参数 0正常 1锁机
        ie_data.u32 = 0; //TODO 读取锁机状态
        break;
    case 0x0007://设备登录密码  6位数字
        ie_data.u32 = 0; //TODO 读取密码
        break;
    case 0x0008://心跳间隔 秒
        ie_data.u32 = 0; //TODO 读取心跳间隔
        break;
    case 0x0009://自动校准配置 0关闭 1打开
        ie_data.u32 = 0; //TODO 读取自动校准状态
        break;
    case 0x000D://静音 0正常 1静音断电保存 2静音断电不保存
        ie_data.u32 = 0; //TODO 读取静音状态
        break;
    case 0x000E://运行模式
        ie_data.u32 = 0; //TODO 读取运行模式
        break;
    case 0x000F://设备控制台透传
        ie_data.u32 = 0; //TODO 读取透传状态
        break;

    default:
        ie_data.u32 = 0;
        LOGT("warn:unknown addr:0x%04X for get\n", addr);
        break;
    }
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, pid); // 使用传入的正确PID

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

    APP_DTU_Cmd_Config_Get(cmd, ie_data.u32, rxBuf[20]); // 传递正确的PID
}
/* 参数获取 */

/* IP设置 */
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

/* IP读取 */
void APP_DTU_Cmd_Ip_Get(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    if (rxBuf[23] == 0 || rxBuf[23] > 4)
    {
        LOG("err:mt get ip!\n");
        return;
    }
    uint8_t chl = rxBuf[23] - 1; //读取位置

    IEEE754 ie_data = {0};
    uint8_t txBuf[128] = {0};
    uint16_t num = 23;

    txBuf[num++] = 0;
    txBuf[num++] = rxBuf[23];

    ie_data.u32 = strlen(g_app_dtu_ip[chl].ip); //ip长度
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    ie_data.u32 = g_app_dtu_ip[chl].port; //端口号
    txBuf[num++] = ie_data.u8_buf[3];
    txBuf[num++] = ie_data.u8_buf[2];
    txBuf[num++] = ie_data.u8_buf[1];
    txBuf[num++] = ie_data.u8_buf[0];

    memcpy(txBuf + num, g_app_dtu_ip[chl].ip, strlen(g_app_dtu_ip[chl].ip)); //ip
    num += strlen(g_app_dtu_ip[chl].ip);

    APP_DTU_Head_Packing(DTU_CMD_TYPE_WRITE, txBuf, (num - 23), cmd, rxBuf[20]);

    uint16_t crc16 = bsp_crc16(txBuf + 23, (num - 23));

    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    APP_DTU_Send(txBuf, num);
}

void APP_DTU_Parse_Read(uint16_t cmd, uint8_t *rxBuf, uint16_t rxLen)
{
    switch (cmd)
    {
    case DTU_CMD_DEVICE_HEARTBEAT:
        if (rxBuf[20] == 99)
        {
            g_dtu_cmd.cnt_status = USR_EOK;
            g_dtu_cmd.response_num = 0; //平台有应答 计数清0
            g_dtu_cmd.net_status = USR_STATE_ON;//dtu连接成功
            if (BSP_CONFIG_Show_Get() == 52)
            {
                LOGT("rmt:ht\n");
            }
        }
        else
        {
            LOGT("err:rmt ht[%d]\n", rxBuf[20]);
            LOG_HEX(rxBuf, rxLen);
        }
        break;
    case DTU_CMD_DEVICE_TIMESYNC:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        g_dtu_cmd.cnt_status = USR_EOK;
        g_dtu_cmd.time_status = USR_TRUE; // 【必需】设置时间同步成功标志，否则OTA无法推进
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:time sync\n");
        }
        break;
    case DTU_CMD_DEVICE_POWE_ON_STATUS:
        g_dtu_cmd.cnt_status = USR_EOK;
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:po\n");
        }
        break;
    case DTU_CMD_DEVICE_OTA_MSG:
        APP_DTU_Ota_Msg_Set(rxBuf, rxLen);
        LOGT("rmt:ota msg\n");
        break;
    case DTU_CMD_DEVICE_OTA_FIRMWARE:
        APP_DTU_Firmware_Backup(rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:ota firmware packet\n");
        }
        break;
    case DTU_CMD_SERVER_GATEWAY:
        g_dtu_cmd.response_num = 0; //平台有应答 计数清0
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:gateway[%d]\n", rxBuf[23]);
        }
        break;
    case DTU_CMD_DEVICE_SIM:
        g_app_rtu_sim.sta = 1;
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:sim info\n");
        }
        break;
    case DTU_CMD_DEVICE_GPS:
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("rmt:gps info\n");
        }
        break;

    case DTU_CMD_SERVER_GSS_DATA_UPLOAD:
        LOGT("rmt:rtd gss\n");
        break;
    case DTU_CMD_SERVER_GSS_ALARM_UPLOAD:
        LOGT("rmt:ent gss\n");
        break;
    case DTU_CMD_SERVER_GSS_CONFIG_INFO:
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
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ht\n");
        }

        break;
    case DTU_CMD_SERVER_TIMESYNC:
        APP_DTU_TimeSync_Set(rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:time sync\n");
        }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_FREQUENCY://0x8002上报间隔设置
        APP_DTU_Cmd_Upload_Interval_Set(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:up interval\n");
        }
        break;

//    case DTU_CMD_SERVER_REMOTE_TRAN:
//        if(BSP_CONFIG_Show_Get()==52)
//        {
//            LOGT("wmt:tran\n");
//        }
//        APP_DTU_Cmd_Remote_Tran(rxBuf,rxLen);
//        break;

    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_SET://0x8003IP设置
        APP_DTU_Cmd_Ip_Set(cmd, rxBuf, rxLen);
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ip set\n");
        }
        break;
    case DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_GET://0x8013IP读取
        APP_DTU_Cmd_Ip_Get(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:ip get\n");
        }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_SET://0x8004系统配置设置
        APP_DTU_Cmd_Config_Set(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOGT("wmt:cfg set\n");
        }
        break;
    case DTU_CMD_SERVER_SYSTEM_CONFIG_GET://0x8014系统配置读取
        APP_DTU_Cmd_Config_Get_Response(cmd, rxBuf, rxLen);
        if (BSP_CONFIG_Show_Get() == 52)
        {
            LOG("wmt:cfg get\n");
        }
        break;
    case DTU_CMD_SERVER_DEVICE_OTA://0x8005 OTA远程升级
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_SUCCESS, rxBuf, rxLen);
        LOGT("ota:remote upgrade triggered by server\n");
        // 使用统一的OTA设置函数（会清除错误日志和计数器）
        APP_DTU_Ota_Set(1);
        break;
    default:
        APP_DTU_Response_Result(cmd, DTU_CMD_RESPONSE_PARSE_FAIL, rxBuf, rxLen);
        break;
    }
}

//数据头打包
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

void APP_DTU_Gps_Check(uint8_t *rxBuf, uint16_t rxLen)
{
    if (strstr((char *)APP_DTU_UART_BUF.rxBuf, "$GNRMC") != NULL)
    {
        g_dtu_cmd.gps_status = bsp_gps_parse((char*)rxBuf, &g_gps_date);
        g_dtu_cmd.gps_enable = 1;
    }
    else if (strstr((char *)rxBuf, "$GNGGA") != NULL)
    {
    }
}

//return 1 数据头正确, 0 未识别
int APP_DTU_Remote_Check_Head(uint8_t *rxBuf, uint16_t rxLen)
{
    int ret = 0;
    uint16_t crc16 = bsp_crc16((uint8_t*)rxBuf, 21); //crc 校验 数据头
    if ((((uint8_t)crc16 & 0xFF) == rxBuf[21]) &&
            ((uint8_t)(crc16 >> 8) == rxBuf[22]))
    {
        ret = 1;
    }

    return ret;
}

//return 1 有数据未处理完, 0 没数据
int APP_DTU_Remote_Check_Body(void)
{
    int ret = 0;
    uint16_t len = g_dtu_rx.rxBuf[2];
    len = (len << 8) + g_dtu_rx.rxBuf[3];

    int deal = 0; //数据是否可用
    uint16_t dtu_remote_index = 23 + len;
    if (len > 0) //有数据 判断crc
    {
        uint16_t crc16 = bsp_crc16(g_dtu_rx.rxBuf + 23, len); //crc 校验 数据

        if ((((uint8_t)crc16 & 0xFF) == g_dtu_rx.rxBuf[len + 23]) &&
                ((uint8_t)(crc16 >> 8) == g_dtu_rx.rxBuf[len + 23 + 1]))
        {
            deal = 1;
            dtu_remote_index += 2; //增加2字节 校验长度
        }
        else
        {
            LOGT("err:body crc [0x%X]\n", crc16);
            if (g_dtu_rx.rxLen > 50)
            {
                LOG_HEX(g_dtu_rx.rxBuf, 50);
            }
            else
            {
                LOG_HEX(g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
            }
        }
    }
    else //没数据 判断是不是心跳
    {
        deal = 1;
    }

    if (deal > 0) //数据有效
    {
        g_dtu_cmd.cmd.u8_buf[1] = g_dtu_rx.rxBuf[17]; //缓存命令编号
        g_dtu_cmd.cmd.u8_buf[0] = g_dtu_rx.rxBuf[18];
        g_dtu_cmd.pid = g_dtu_rx.rxBuf[20];      //缓存包序列
        
        // 设置响应状态（与HC32保持一致）
        g_dtu_cmd.response_status = USR_TRUE;
        
        LOGT("ota:rx cmd=0x%04X, type=%c, pid=%d\n", 
             g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf[1], g_dtu_cmd.pid);
        
        if (g_dtu_rx.rxBuf[1] == 'R')                       //判断为设备主动发送后平台的回复
        {
            APP_DTU_Parse_Read(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }
        else if (g_dtu_rx.rxBuf[1] == 'W')          //判断为平台主动发送给设备的命令
        {
            APP_DTU_Parse_Write(g_dtu_cmd.cmd.u16, g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        }

        if (g_dtu_rx.rxLen > dtu_remote_index) //判断是否有剩余数据
        {
            g_dtu_rx.rxLen -= dtu_remote_index;
            memcpy(g_dtu_rx.rxBuf, g_dtu_rx.rxBuf + dtu_remote_index, g_dtu_rx.rxLen);
            ret = 1;
        }
    }

    return ret;
}

//return 0 未识别数据, 1 数据已解析
int APP_DTU_Remote_Check(void)
{
    int ret = 0;
    while (g_dtu_rx.rxLen > 22)
    {
        ret = APP_DTU_Remote_Check_Head(g_dtu_rx.rxBuf, g_dtu_rx.rxLen);
        if (ret == 1) //头正确
        {
            if (APP_DTU_Remote_Check_Body() == 0) //已处理完,退出
            {
                break;
            }
        }
        else //不可用包 退出
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
        //多通道数据解析
        g_dtu_rx = APP_RTU_AT_Rx_Chl(APP_DTU_UART_BUF.rxBuf, APP_DTU_UART_BUF.rxLen);
        if (g_dtu_rx.rxLen > 0)
        {
            APP_DTU_Remote_Check();
        }
    }
}

//发送实时数据/事件数据
void APP_DTU_Send_DTURealTimeRecord(void)
{
    uint8_t txBuf[1024] = {0};
    uint16_t num = 23;
    uint16_t pack_len = 1;

    IEEE754 aucData1 ;

    txBuf[num++] = 0; //默认值 第一版    //23
    txBuf[num++] = pack_len >> 8; //数据数量
    txBuf[num++ ] = pack_len; //数据数量

    for (u8 i = 0; i < pack_len; i++)
    {
        BSP_RTC_Get(&g_bsp_rtc);
        txBuf[num++] = (2000 + g_bsp_rtc.year) >> 8; //年
        txBuf[num++] = (2000 + g_bsp_rtc.year) & 0xFF;
        txBuf[num++] = g_bsp_rtc.month;
        txBuf[num++] = g_bsp_rtc.day;
        txBuf[num++] = g_bsp_rtc.hour;
        txBuf[num++] = g_bsp_rtc.minute;
        txBuf[num++] = g_bsp_rtc.second;


        aucData1.flt = 0;//位置实际值 厘米为单位
        txBuf[num++] = aucData1.u8_buf[3]; //num33
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];


        aucData1.u32 = 0;//位置ad值
        txBuf[num++] = aucData1.u8_buf[3]; //num37
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = 0;//实时速度  200ms的变化量，还需要加K值
        txBuf[num++] = aucData1.u8_buf[3]; //num41
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = 1 >> 8; 
        txBuf[num++] = 1 & 0xff;

        aucData1.flt = 2048;//电磁1的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = 1 >> 8; //电磁2的AD信号
        txBuf[num++] = 1 & 0xff;


        aucData1.flt = 2048;//电磁2的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = 1 >> 8; //电磁3的AD信号
        txBuf[num++] = 1 & 0xff;


        aucData1.flt = 2048;//电磁3的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = 1 >> 8; //电磁4的AD信号
        txBuf[num++] = 1 & 0xff;

        aucData1.flt = 2048;//电磁4的电压信号

        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        aucData1.flt = 0;//最大损伤程度
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        txBuf[num++] = 0; //损伤程度报警状态
        txBuf[num++] = 0;

        txBuf[num++] = 0; //轻微损伤数量
        txBuf[num++] = 0;

        txBuf[num++] = 0; //严重损伤数量
        txBuf[num++] = 0;

        txBuf[num++] = 200 >> 8; //绳总长
        txBuf[num++] = 200 & 0xff;

        txBuf[num++] = 100 >> 8; //损伤度-健康度
        txBuf[num++] = 100 & 0xff;
        //最大损伤位置
        txBuf[num++] = 0 ; //最大损伤位置
        txBuf[num++] = 0 ;

        // ===== 新增字段：实时阈值和滤波数据 =====
        // 字节62-65: 均值偏差实时阈值
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节66-69: 单传感器实时阈值
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节70-73: 方差异常实时阈值
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节74-77: 趋势异常实时数据
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节78-81: 综合判断实时阈值
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

        // 字节82-85: 滤波实时数据
        aucData1.flt = 0;
        txBuf[num++] = aucData1.u8_buf[3];
        txBuf[num++] = aucData1.u8_buf[2];
        txBuf[num++] = aucData1.u8_buf[1];
        txBuf[num++] = aucData1.u8_buf[0];

    }
    APP_DTU_Head_Packing(DTU_CMD_TYPE_READ, txBuf, (num - 23), DTU_CMD_SERVER_GSS_DATA_UPLOAD, 0);

    // 计算CRC16
    uint16_t crc16 = bsp_crc16(txBuf + 23, num - 23);
    txBuf[num++] = (uint8_t)crc16 & 0xFF;
    txBuf[num++] = (uint8_t)(crc16 >> 8);

    // 发送数据
    APP_DTU_Send(txBuf, num);

    LOGT("send config info to remote [%d]\n", num);
    LOG_HEX(txBuf, num);
}


int  APP_DTU_Remote_Cnt_Sta_Get(void)
{
    return g_dtu_cmd.net_status;
}

void APP_DTU_Init(void)
{
    APP_RTU_AT_Init();//建立了 APP_RTU_AT_Config_Handle APP_RTU_AT_Config_Handle_Err   2个回调函数
    APP_DTU_Status_Reset();
    APP_DTU_Remote_Head_Init();//数据头打包
    BSP_DELAY_MS(2000);

}

void APP_DTU_Handle(void)
{
    APP_DTU_Rec_Handle();
    APP_DTU_Ota_Handle();  // 添加OTA处理
}

/*****END OF FILE****/


