#ifndef __APP_DTU_H
#define __APP_DTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

//总平台参数
#define  REMOTE_SN         2900000
#define  REMOTE_MODEL      0x1E
#define  REMOTE_VER        0x5B

#define DTU_CMD_TYPE_READ    'R'
#define DTU_CMD_TYPE_WRITE   'W'

#define DTU_CMD_RESPONSE_SUCCESS            0
#define DTU_CMD_RESPONSE_FAIL               1
#define DTU_CMD_RESPONSE_TIMEOUT            2
#define DTU_CMD_RESPONSE_PARSE_FAIL     0xFF

#define DTU_CMD_DEVICE_HEARTBEAT                                            0x0000
#define DTU_CMD_DEVICE_TIMESYNC                                             0x0001
#define DTU_CMD_DEVICE_POWE_ON_STATUS                           0x0002
#define DTU_CMD_DEVICE_OTA_MSG                                          0x0003
#define DTU_CMD_DEVICE_OTA_FIRMWARE                             0x0004
#define DTU_CMD_DEVICE_GPS                                              0x0005

#define DTU_CMD_DEVICE_SIM                                                0x0007

#define DTU_CMD_SERVER_TOWER_BASE_DATA_UPLOAD                                     0x1d00
#define DTU_CMD_SERVER_TOWER_BASE_ALARM_UPLOAD                                       0x1d09

#define DTU_CMD_SERVER_GSS_DATA_UPLOAD                                    0x1e00
#define DTU_CMD_SERVER_GSS_ALARM_UPLOAD                                      0x1e02
#define DTU_CMD_SERVER_GSS_CONFIG_INFO                                      0x1e04

#define DTU_CMD_SERVER_HEARTBEAT                                            0x8000
#define DTU_CMD_SERVER_TIMESYNC                                             0x8001
#define DTU_CMD_SERVER_DATA_UPLOAD_FREQUENCY                    0x8002
#define DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_SET              0x8003
#define DTU_CMD_SERVER_DATA_UPLOAD_ADDRESS_GET              0x8013
#define DTU_CMD_SERVER_SYSTEM_CONFIG_SET                            0x8004
#define DTU_CMD_SERVER_SYSTEM_CONFIG_GET                            0x8014
#define DTU_CMD_SERVER_DEVICE_OTA                                           0x8005
#define DTU_CMD_SERVER_REMOTE_DATA                                          0x8006

#define DTU_CMD_SERVER_REMOTE_TRAN                              0x8007
#define DTU_CMD_SERVER_GATEWAY                                            0x6000



enum
{
    SENSOR_TP_BO1 = 1,//重量
    SENSOR_TP_BO2,   //电位器（高度、幅度、回转、行程）
    SENSOR_TP_BO3,   //增量编码器（升降机高度）
    SENSOR_TP_BO4,   //绝对值编码器（回转）
    SENSOR_TP_BO5,   //电子罗盘
    SENSOR_TP_BO6,   //风速
    SENSOR_TP_BO7,   //倾角X
    SENSOR_TP_BO8,   //倾角Y
    SENSOR_TP_BO9,   //倾角Z
    SENSOR_TP_BO10,  //人脸
    SENSOR_TP_BO11,  //板载电池
    SENSOR_TP_BO12,  //红外测距
    SENSOR_TP_BO13,  //行程差
    SENSOR_TP_BO14,  //开关状态
    SENSOR_TP_BO15,  //时长
    SENSOR_TP_BO16,  //命令
    SENSOR_TP_BO17,  //系统状态
    SENSOR_TP_BO18,  //静态数据（浮点数）
    SENSOR_TP_BO19,  //静态数据（字符串）
    SENSOR_TP_BO20,  //静态数据（整数）
    SENSOR_TP_BO21,  //表面应力
    SENSOR_TP_BO22,  //18B20温度
    SENSOR_TP_BO23,  //拉绳传感器
    SENSOR_TP_BO24,  //继电器
    SENSOR_TP_BO25,  //GPS定位数据
    SENSOR_TP_BO26,  //电阻桥锚力计
    SENSOR_TP_BO27,  //速度


};
#define DTU_EVENT_MAX                                                                 10

enum
{
    POWER_ON_DTU_STEP0 = 0,    // 心跳
    POWER_ON_DTU_STEP1,        // 时间同步
    POWER_ON_DTU_STEP2,        // 上报开机信息
    POWER_ON_DTU_STEP3,        // OTA: 请求固件信息
    POWER_ON_DTU_STEP4,        // OTA: 下载固件分包
    POWER_ON_DTU_STEP5,
    POWER_ON_DTU_STEP6,
    POWER_ON_DTU_STEP7,

    POWER_ON_DTU_STEP_MAX,
};

enum
{
    DTU_CNT_STEP0 = 0,
    DTU_CNT_STEP1,
    DTU_CNT_STEP2,
    DTU_CNT_STEP3,

    DTU_CNT_STEP_MAX,
};
typedef struct
{
    uint16_t sensor_type : 12;     //传感器数据类型
    uint16_t sensor_num  : 4;     //高4位表示通道号
} dtu_sensor_type_def;


typedef struct
{
    uint32_t uint_sn;      //定义设备SN前7位
    uint32_t uint_model;   //定义设备型号代码
    uint32_t uint_ver;       //定义设备版本号
} dtu_remote_def;
extern dtu_remote_def  g_dtu_remote;

typedef struct
{
    uint8_t      pid; //包序号
    bsp_un16_def cmd; //指令
    uint8_t      head[20]; //包头

    uint8_t   response_cmd;   //发送指令记录
    uint8_t   send_cmd;       //发送指令

    uint32_t  response_num;    //应答计数,信号状态使用
    uint64_t  time_num;    //秒计数
    uint32_t  send_num;         //发送计数

    uint8_t   power_on_status;  //上电状态

    uint8_t   gps_enable;//是否支持定位
    uint8_t   gps_status;//定位状态
    uint32_t  gps_send_interval;    //gps发送间隔

    uint8_t   net_status;//网络信号状态
    uint8_t   cnt_status;//上电连接应答状态

    uint8_t cnt_rtd_i;
    uint8_t cnt_rtd_j;
    uint8_t cnt_rtd_sta;
    
    // OTA相关字段
    uint16_t  cmd_last;          // 上次发送的命令
    uint8_t   dtu_step;          // OTA步骤
    uint8_t   send_times;        // 发送次数(重试计数)
    uint8_t   time_status;       // 时间同步状态
    uint8_t   response_status;   // 响应状态
} dtu_cmd_def;

extern dtu_cmd_def g_dtu_cmd;

typedef struct
{
    uint32_t normal_interval;
    uint32_t work_interval;
    uint32_t send_interval;

} dtu_remote_cmd_def;
extern dtu_remote_cmd_def g_dtu_remote_cmd;

typedef struct
{
    uint8_t event_buf[128];
    uint8_t event_num;
} event_buf_def;

typedef struct
{
    float       position_data_real;  //位置实际值
    uint32_t    position_data_ad;    //位置ad值
    float       real_speed;        //实时速度
    uint16_t    hall_ad[8];        //实时的ad值
    float   hall_v[8];

    uint16_t    degree_of_damage;  //损伤程度
    uint16_t    alarm;              //损伤程度报警状态   0无报警  1预警 2报警
    uint32_t    Total_meters;      //总里程
    uint32_t    position_zero_point; //位置标定零点
    uint8_t     run_direction;     //运行方向 0=停止 1=向上 2=向下

    // 位置量程和信号上下限
    uint32_t    position_range_upper;  //位置量程上限
    uint32_t    position_range_lower;  //位置量程下限
    uint32_t    position_signal_upper; //位置信号上限
    uint32_t    position_signal_lower; //位置信号下限
    uint16_t    Threshold_set1; //轻微损伤电压标定
    uint16_t    Threshold_set2; //严重损伤电压标定
    uint16_t    Threshold_set3; //上半波宽度标定
    uint16_t    Threshold_set4; //下半波宽度标定
    // 线性标定参数（参考程序设计实现）
    float       position_slope;        //位置线性标定斜率
    float       position_offset;       //位置线性标定截距
    uint16_t    total_length;
    int16_t    max_damage_position; //最大损伤位置
} gss_device;

typedef struct
{
    bsp_rtc_def alarmtime;          //报警时的时间

    uint8_t     trig_case;

    float       position_data_real;  //报警时高度实际值
    uint32_t    position_data_ad;    //报警时高度ad值
    float       real_speed;        //报警时实时速度
    uint16_t    hall_ad[8];        //报警时的ad值
    uint32_t    hall_v[8];         //报警时的电压值
    uint16_t    degree_of_damage;  //损伤程度
    uint16_t    alarm;              //损伤程度报警状态  float32  0 正常 1轻微损伤 2严重损伤

    // 位置信息字段
    uint32_t    position_data;      //报警时的位置数据
    uint8_t     position_valid;     //位置数据是否有效 0=无效 1=有效


} gss_device_alarm_stat;


enum
{
    REMOTE_SENSOR_WEIGHT = 1, //重量
    REMOTE_SENSOR_POTENTIOMETER,//电位器（高度、幅度、回转、行程）
    REMOTE_SENSOR_ENCODER,//增量编码器（升降机高度）
    REMOTE_SENSOR_ENCODER_ABSOLUTE,//绝对值编码器（回转）
    REMOTE_SENSOR_COMPASS,//电子罗盘
    REMOTE_SENSOR_WIND,//风速

};

void APP_DTU_Head_Packing(uint8_t type, uint8_t *txBuf, uint16_t not_head_len, uint16_t cmd, uint8_t pid);
void APP_DTU_Tran_Remote(uint8_t *buf, uint16_t len);
void APP_DTU_Cmd_Remote_Tran_Response(uint8_t *json, uint32_t json_len);

void APP_DTU_Status_Reset(void);
void APP_DTU_Send_Buf(uint8_t *buffer, uint16_t size);
void APP_DTU_Send(uint8_t *buffer, uint16_t size);
void APP_DTU_Handle(void);
void APP_DTU_Init(void);
int  APP_DTU_Remote_Cnt_Sta_Get(void);

// OTA相关函数
void APP_DTU_Ota_Set(uint8_t opt);        // OTA触发设置
void APP_DTU_Boot_Reset(void);            // BOOT复位（清除状态并重启）
void APP_DTU_Send_Get_Firmware_Msg(void);
void APP_DTU_Ota_Msg_Set(uint8_t *rxBuf, uint16_t rxLen);
void APP_DTU_Firmware_Backup_Ready(void);
void APP_DTU_Send_Get_Firmware_Sub(uint32_t addr, uint32_t len);
void APP_DTU_Send_Get_Firmware(void);     // 获取远程固件（包装函数）
void APP_DTU_Firmware_Backup(uint8_t *rxBuf, uint16_t rxLen);
uint8_t APP_DTU_Firmware_Head_Check(uint8_t *rxBuf);
int APP_DTU_Send_Get_Firmware_Msg_Sub(void);
uint8_t APP_DTU_Ota_Connect_Handle(void);     // OTA连接和状态机处理（区别于开机联网的APP_DTU_Connect_Remote_Handle）
void APP_DTU_Ota_Handle(void);            // OTA主处理函数（循环调用）
void APP_DTU_Send_DTURealTimeRecord(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DTU_H */

/*****END OF FILE****/
