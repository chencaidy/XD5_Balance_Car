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
        case CONTROL:
        {
            if (16 == btn->buttonID)    //一档按钮
            {
                //TODO
                Brake.Count = 0;
                Brake.StopDelay = 300;

                Barrier.Delay = 1000;

                Circle.Count = 0;
                Cross.Count = 0;
                Process.normpdf = normpdf45;

                img_start_signal();
                Speed.I_Error_Start = 1000;
                Speed.I_Limit_PWM_max = 20;
                Speed.I_Limit_PWM_min = -20;
                Speed.PWM_Integral = 20.f;
                Speed.Goal = 4250.f;

                Direction.P = 0.98f;
                Direction.D = 0.058f;

                PRINTF("HMI Message: Gear 1 Press!\r\n");
            }
            if (17 == btn->buttonID)    //二档按钮
            {
                //TODO
                Brake.Count = 0;
                Brake.StopDelay = 200;

                Circle.Count = 0;
                Cross.Count = 0;
                Process.normpdf = normpdf43;

                img_start_signal();
                Speed.Goal = 4650.f;
                Speed.I_Error_Start = 1000;
                Speed.I_Limit_PWM_max = 10;
                Speed.I_Limit_PWM_min = -10;
                Speed.PWM_Integral = 10.f;

                Direction.P = 0.98f;
                Direction.D = 0.058f;

                PRINTF("HMI Message: Gear 2 Press!\r\n");
            }
            if (18 == btn->buttonID)    //三档按钮
            {
                //TODO
                Brake.Count = 0;
                Brake.StopDelay = 200;

                Circle.Count = 0;
                Cross.Count = 0;
                Process.normpdf = normpdf40;
                Process.Delay = 2000;

                img_start_signal();
                Speed.Goal = 5070.f;
                Speed.I_Error_Start = 500;
                Speed.I_Limit_PWM_max = 5;
                Speed.I_Limit_PWM_min = -5;
                Speed.PWM_Integral = 5.f;

                Direction.P = 0.98f;
                Direction.D = 0.058f;

                PRINTF("HMI Message: Gear 3 Press!\r\n");
            }

            if (21 == btn->buttonID)    //爆档按钮
            {
                //TODO
                Brake.Count = 0;
                Brake.StopDelay = 200;

                Circle.Count = 0;
                Cross.Count = 0;
                Process.normpdf = normpdf35;
                Process.Delay = 2000;

                img_start_signal();
                Speed.Goal = 5350.f;
                Speed.I_Error_Start = 500;
                Speed.I_Limit_PWM_max = 5;
                Speed.I_Limit_PWM_min = -5;
                Speed.PWM_Integral = 5.f;

                Direction.P = 0.98f;
                Direction.D = 0.058f;

                PRINTF("HMI Message: Gear Boom Press!\r\n");
            }
            break;
        }

        case SENSOR:
        {
            if (12 == btn->buttonID)    //校准按钮
            {
                if (kStatus_Success == IMU_Calibrate(&imuBias))
                {
                    Blackbox_WriteConf("/CONFIG/IMU.CF", &imuBias, sizeof(imuBias));
                    IMU_SetBias(imuBias);
                }
            }
            break;
        }

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

        case PID:
        {
            if (6 == btn->buttonID)     //500ms障碍按钮
                Barrier.Delay = 500;
            if (8 == btn->buttonID)     //1000ms障碍按钮
                Barrier.Delay = 1000;
            if (9 == btn->buttonID)     //1500ms障碍按钮
                Barrier.Delay = 1500;
            if (11 == btn->buttonID)     //+100ms障碍按钮
                Barrier.Delay += 100;
            if (12 == btn->buttonID)     //-100ms障碍按钮
                Barrier.Delay -= 100;

            if (14 == btn->buttonID)     //+0.1L障碍按钮
                Barrier.Offset_Goal_L += 0.1;
            if (15 == btn->buttonID)     //-0.1L障碍按钮
                Barrier.Offset_Goal_L -= 0.1;
            if (17 == btn->buttonID)     //+0.1R障碍按钮
                Barrier.Offset_Goal_R += 0.1;
            if (18 == btn->buttonID)     //-0.1R障碍按钮
                Barrier.Offset_Goal_R -= 0.1;

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
        case CONTROL:
        {
            if (9 == silder->widgetID)     //十字按钮
            {
                //TODO
                PRINTF("HMI Message: Cross Value is %d\r\n", silder->val);
                Cross.ON = silder->val;
            }
            if (10 == silder->widgetID)     //小S按钮
            {
                //TODO
                PRINTF("HMI Message: Circle Value is %d\r\n", silder->val);
            }
            if (11 == silder->widgetID)     //坡道按钮
            {
                //TODO
                PRINTF("HMI Message: Block Value is %d\r\n", silder->val);
                Slope.ON = silder->val;
            }
            if (12 == silder->widgetID)     //刹车按钮
            {
                PRINTF("HMI Message: Slope Value is %d\r\n", silder->val);
                Brake.ON = silder->val;
            }
            if (13 == silder->widgetID)     //失控保护按钮
            {
                PRINTF("HMI Message: Break Value is %d\r\n", silder->val);
                Failsafe.ON = silder->val;
            }

            /* 圆环方向 */
            if (3 == silder->widgetID)
                Circle.Dir[0] = silder->val;
            if (4 == silder->widgetID)
                Circle.Dir[1] = silder->val;
            if (5 == silder->widgetID)
                Circle.Dir[2] = silder->val;
            if (6 == silder->widgetID)
                Circle.Dir[3] = silder->val;
            if (7 == silder->widgetID)
                Circle.Dir[4] = silder->val;
            if (20 == silder->widgetID)
                Circle.Dir[5] = silder->val;

            break;
        }

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

        case PID:
        {
            if (3 == silder->widgetID)     //直立环按钮
                Angle.ON = silder->val;
            if (4 == silder->widgetID)     //速度环按钮
                Speed.ON = silder->val;
            if (5 == silder->widgetID)     //角度环按钮
                Direction.ON = silder->val;
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
 * @brief  用户特殊格式处理事件
 * @retval -1,失败 0,成功
 */
static int8_t userHandle(uint8_t *buf)
{
    hmiPack_t *packge = NULL;

    if (NULL == buf)
        return -1;
    packge = (hmiPack_t *) buf;

    switch (packge->cmd)
    {
        //TODO:用户自定义

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
        /* 不满足标准格式，进入用户特殊格式处理事件 */
        default:
        {
            userHandle(buf);
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
        case CONTROL:
        {
            HMI_InsertData("bt0.val=%d", Cross.ON);
//            HMI_InsertData("bt1.val=%d", );
//            HMI_InsertData("bt2.val=%d", );
            HMI_InsertData("bt3.val=%d", Brake.ON);
            HMI_InsertData("bt4.val=%d", Failsafe.ON);
            HMI_InsertData("bt_c1.val=%d", Circle.Dir[0]);
            HMI_InsertData("bt_c2.val=%d", Circle.Dir[1]);
            HMI_InsertData("bt_c3.val=%d", Circle.Dir[2]);
            HMI_InsertData("bt_c4.val=%d", Circle.Dir[3]);
            HMI_InsertData("bt_c5.val=%d", Circle.Dir[4]);
            HMI_InsertData("bt_c6.val=%d", Circle.Dir[5]);
            break;
        }

        case SENSOR:
        {
            HMI_InsertData("n0.val=%d", (int) (sensor.Pitch * 100.f));
            HMI_InsertData("n1.val=%d", (int) (sensor.GyroX * 100.f));
            HMI_InsertData("n2.val=%d", (int) (sensor.GyroZ * 100.f));
            break;
        }

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

        case PID:
        {
            HMI_InsertData("bt0.val=%d", Angle.ON);
            HMI_InsertData("bt1.val=%d", Speed.ON);
            HMI_InsertData("bt2.val=%d", Direction.ON);
            HMI_InsertData("t_ba.txt=\"%d\"", Barrier.Delay);
            HMI_InsertData("t_of.txt=\"L%d|R%d\"",
                    (int) (Barrier.Offset_Goal_L * 10),
                    (int) (Barrier.Offset_Goal_R * 10));
            break;
        }

        default:
        {
            break;
        }
    }

    HMI_SendData();
}
