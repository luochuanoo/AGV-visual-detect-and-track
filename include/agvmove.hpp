/*
 * @Descripttion: AGV 运动控制
 * @version: 
 * @Author: LuoChuan
 * @Date: 2020-08-03 15:37:22
 * @LastEditors: luochuan
 * @LastEditTime: 2020-09-16 16:59:23
 */

#ifndef MOVE_H
#define MOVE_H

typedef enum AGV_STATUS_LIST
{
    AGV_STOP            =0, 
    AGV_FORWARD         =1, 
    AGV_BACKWARD        =2, 
    AGV_LEFT            =3, 
    AGV_RIGHT           =4, 
    AGV_LEFT_FRONT      =5, 
    AGV_RIGHT_FRONT     =6, 
    AGV_LEFT_REAR       =7, 
    AGV_RIGHT_REAR      =8,
} AGV_STATUS_LIST;
typedef int AGV_STATUS;

class AGVMove
{
private:
	float delta_x;      // 设置一个平台振动误差范围，减少因振动产生的影响
	float delta_y;
    int leftSpeed;      // 左右轮的转速
    int rightSpeed;

public:
    /**
     * @description: 构造函数
     * @param {type} 
     * @return {type} 
     */
    AGVMove();

    /**
     * @description: 析构函数
     * @param {type} 
     * @return {type} 
     */
    virtual ~AGVMove() {};

    /**
     * @description: 设置 AGV 振动误差
     * @param _x  x 方向的振动误差
     * @param _y  y 方向的振动误差
     * @return void 
     */
    void setVibrationError(float _x, float _y);
    
    /**
     * @description: 判断目标在小车前方的位置，确定小车的状态
     * @param center1X 当前帧目标中心点 x 坐标
     * @param center1Y 当前帧目标中心点 y 坐标
     * @param center2X 图像中心点 x 坐标
     * @param center2Y 图像中心点 y 坐标
     * @return AGV_STATUS 
     */
    void checkStatus(float center1X, float center1Y, float center2X, float center2Y);

    /**
     * @description: 通过增减左右轮的速度，改变左右轮的转向和速度，实现 AGV 的控制
     * @param left
     * @param right
     * @return void
     */
    void setSpeed(int left, int right);

    /**
     * @description: 向串口设备发送指令
     * @param fd 
     * @return void 
     */
    void make_packet_and_send(int fd);

    /**
     * @description: 根据状态选择运动模式
     * @param status AGV 小车的状态
     * @return void 
     */
    void move(AGV_STATUS status);

    /**
     * @description: 停止
     * @param {type} 
     * @return void
     */
    void goStop();

    /**
     * @description: 前进
     * @param {type} 
     * @return void
     */
    void goForward();

    /**
     * @description: 后退
     * @param {type} 
     * @return void
     */
    void goBackward();

    /**
     * @description: 左转
     * @param {type} 
     * @return void
     */
    void goLeft();

    /**
     * @description: 右转
     * @param {type} 
     * @return void
     */
    void goRight();

    /**
     * @description: 左前行驶
     * @param {type} 
     * @return void
     */
    void goLeftFront();

    /**
     * @description: 右前行驶
     * @param {type} 
     * @return void
     */
    void goRightFront();

    /**
     * @description: 左后行驶
     * @param {type} 
     * @return void
     */
    void goLeftRear();

    /**
     * @description: 右后行驶
     * @param {type} 
     * @return void
     */
    void goRightRear();
};

#endif