/**
  ******************************************************************************
  * File Name          : hmiHandle.c
  * Description        : HMI串口屏消息处理文件
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "common.h"

#include "fsl_common.h"
#include "fsl_debug_console.h"

/* Variables -----------------------------------------------------------------*/
static pageID_e thisPage;

/**
 * @brief  按钮处理事件
 * @retval -1,失败 0,成功
 */
static int8_t buttonHandle(uint8_t *buf)
{
    hmiButton_t *btn = NULL;
    
    if (NULL == buf)
        return -1;
    btn = (hmiButton_t *) buf;
    
    switch (thisPage)
    {
        case CAMERA:
        {
            if (15 == btn->buttonID)    //保存按钮
            {
                Blackbox_WriteConf("/CONFIG/CAM.CF", &camera, sizeof(camera));
                CAM_UpdateProfile(&camera);
            }
            if (4 == btn->buttonID)     //50hz按钮
                camera.FPS = 50;
            if (5 == btn->buttonID)     //75hz按钮
                camera.FPS = 75;
            if (6 == btn->buttonID)     //112hz按钮
                camera.FPS = 112;
            if (7 == btn->buttonID)     //150hz按钮
                camera.FPS = 150;
            break;
        }

        case BLACKBOX:
        {
            if (6 == btn->buttonID)     //启动按钮
            {
                Blackbox_Start();
                PRINTF("\r\n--> Recording...\r\n");
            }
            if (7 == btn->buttonID)     //停止按钮
            {
                Blackbox_Stop();
                PRINTF("\r\n--> Record finished.\r\n");
            }
            if (8 == btn->buttonID)     //复位按钮
                Blackbox_Reset();
            if (9 == btn->buttonID)     //格式化按钮
                Blackbox_Format();
            break;
        }

        default:
        {
            break;
        }
    }
    
    return 0;
}

/**
 * @brief  滑块处理事件
 * @retval -1,失败 0,成功
 */
static int8_t silderHandle(uint8_t *buf)
{
    hmiUser_t *silder = NULL;

    if (NULL == buf)
        return -1;
    silder = (hmiUser_t *) buf;

    switch (thisPage)
    {
        case CAMERA:
        {
            if (10 == silder->widgetID)     //对比度滑块
                camera.CNST = silder->val;
            if (14 == silder->widgetID)     //曝光度滑块
                camera.AEC = silder->val;
            break;
        }

        default:
        {
            break;
        }
    }

    return 0;
}

/**
 * @brief  双态按钮处理事件
 * @retval -1,失败 0,成功
 */
static int8_t doubleHandle(uint8_t *buf)
{
    hmiUser_t *silder = NULL;

    if (NULL == buf)
        return -1;
    silder = (hmiUser_t *) buf;

    switch (thisPage)
    {
        case CAMERA:
        {
            if (16 == silder->widgetID)     //自动曝光按钮
                camera.AutoAEC = silder->val;
            if (17 == silder->widgetID)     //自动白平衡按钮
                camera.AutoAWB = silder->val;
            if (18 == silder->widgetID)     //自动增益按钮
                camera.AutoAGC = silder->val;
            break;
        }

        default:
        {
            break;
        }
    }

    return 0;
}

/**
 * @brief  页面处理事件
 * @retval -1,失败 0,成功
 */
static int8_t pageHandle(uint8_t *buf)
{
    hmiPage_t *page = NULL;

    if (NULL == buf)
        return -1;
    page = (hmiPage_t *) buf;

    thisPage = page->pageID;
    HMI_TxMsgHandle();

    return 0;
}

/**
 * @brief  接收消息处理总事件
 * @retval -1,失败 0,成功
 */
int8_t HMI_RxMsgHandle(uint8_t *buf)
{
    hmiPack_t *packge = NULL;
    
    if (NULL == buf)
        return -1;
    packge = (hmiPack_t *) buf;
    
    switch (packge->cmd)
    {
        case BUTTON_RTN:
        {
            buttonHandle(packge->data);
            break;
        }

        case PAGE_RTN:
        {
            pageHandle(packge->data);
            break;
        }

        case SILDER_RTN:
        {
            silderHandle(packge->data);
            break;
        }

        case DOUBLE_RTN:
        {
            doubleHandle(packge->data);
            break;
        }

        default:
        {
            break;
        }
    }
    
    return 0;
}

/**
 * @brief  发送消息处理总事件
 * @retval None
 */
void HMI_TxMsgHandle(void)
{
    switch(thisPage)
    {
        case CAMERA:
        {
            Blackbox_ReadConf("/CONFIG/CAM.CF", &camera, sizeof(camera));
            HMI_InsertData("fps.val=%d", camera.FPS);
            HMI_InsertData("cnst.val=%d", camera.CNST);
            HMI_InsertData("h_cnst.val=%d", camera.CNST);
            HMI_InsertData("ace.val=%d", camera.AEC);
            HMI_InsertData("h_ace.val=%d", camera.AEC);
            HMI_InsertData("bt_ace.val=%d", camera.AutoAEC);
            HMI_InsertData("bt_awb.val=%d", camera.AutoAWB);
            HMI_InsertData("bt_agc.val=%d", camera.AutoAGC);
            break;
        }

        case BLACKBOX:
        {
            HMI_InsertData("free.txt=\"(Free: %dMB)\"", Blackbox_GetFree());
            break;
        }

        case REMOTE:
        {
            HMI_InsertData("n0.val=%d", rcInfo.ch[0]);
            HMI_InsertData("n1.val=%d", rcInfo.ch[1]);
            HMI_InsertData("n2.val=%d", rcInfo.ch[2]);
            HMI_InsertData("n3.val=%d", rcInfo.ch[3]);
            HMI_InsertData("n4.val=%d", rcInfo.ch[4]);
            HMI_InsertData("n5.val=%d", rcInfo.ch[5]);
            HMI_InsertData("n6.val=%d", rcInfo.ch[6]);
            HMI_InsertData("n7.val=%d", rcInfo.ch[7]);
            HMI_InsertData("n8.val=%d", rcInfo.ch[8]);
            HMI_InsertData("n9.val=%d", rcInfo.ch[9]);
            break;
        }

        default:
        {
            break;
        }
    }

    HMI_SendData();
}
