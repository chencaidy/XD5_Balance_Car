/**
  ******************************************************************************
  * File Name          : hmiHandle.h
  * Description        : HMI串口屏消息处理头文件
  ******************************************************************************
  */
  
#ifndef __HANDLE_HMI_H
#define __HANDLE_HMI_H

#include <stdint.h>

/* 命令类型枚举 */
typedef enum
{
/* 串口指令执行成功或失败的通知格式 */
    CMD_ERR     = 0x00,
    CMD_PASS    = 0x01,
    BTN_ERR     = 0x02,
    PAGE_ERR    = 0x03,
    IMG_ERR     = 0x04,
    FONT_ERR    = 0x05,
    BAUD_ERR    = 0x11,
    SCOPE_ERR   = 0x12,
    VALUE_ERR   = 0x1A,
    MATH_ERR    = 0x1B,
    ASSIGN_FAIL = 0x1C,
    EEPROM_FAIL = 0x1D,
    VNUM_ERR    = 0x1E,
    IO_FAIL     = 0x1F,
    CHAR_ERR    = 0x20,
    LENGTH_ERR  = 0x23,
    
/* 其他数据类型返回 */
    BUTTON_RTN  = 0x65,
    PAGE_RTN    = 0x66,
    TOUCH_RTN   = 0x67,
    SLPTAP_RTN  = 0x68,
    STRING_RTN  = 0x70,
    NUM_RTN     = 0x71,
    SLEEP_RTN   = 0x86,
    WAKE_RTN    = 0x87,
    BOOT_RTN    = 0x88,
    UPDATE_RTN  = 0x89,
    
/* 自定义数据类型返回 */
    SILDER_RTN  = 0xA0,     //滑块事件
    DOUBLE_RTN  = 0xA1,     //双态按钮事件

    IMG_CIRCLE_RTN = 0xB1,  //圆环转向控制专用事件

} cmdBit_e;

/* 页面ID枚举 */
typedef enum
{
    BOOT        = 0,
    MAIN        = 1,
    CONTROL     = 2,
    SENSOR      = 3,
    CAMERA      = 4,
    BLACKBOX    = 5,
    REMOTE      = 6,
    PID         = 7,
    
} __attribute__ ((packed)) pageID_e;

/** HMI数据通用结构 */
typedef struct {
    cmdBit_e  cmd;           //返回数据第一位-命令位
    uint8_t   data[32];      //数据位
    
} __attribute__ ((packed)) hmiPack_t;

/** HMI触摸热区事件返回 */
typedef struct {
    pageID_e  pageID;
    uint8_t   buttonID;
    uint8_t   type;
    uint8_t   end[3];
    
} __attribute__ ((packed)) hmiButton_t;

/** HMI当前页面ID号返回 */
typedef struct {
    pageID_e  pageID;
    uint8_t   end[3];
    
} __attribute__ ((packed)) hmiPage_t;

/** HMI自定义事件返回格式 */
typedef struct {
    uint32_t  widgetID;
    uint32_t  val;
    uint8_t   end[3];

} __attribute__ ((packed)) hmiUser_t;


int8_t HMI_RxMsgHandle(uint8_t *buf);
void HMI_TxMsgHandle(void);

#endif /* __HANDLE_HMI_H */
