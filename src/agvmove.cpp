/*
 * @Description: AGV 运动控制
 * @Author: luochuan
 * @Date: 2020-08-06 14:03:01
 * @LastEditTime: 2020-09-16 16:48:43
 * @LastEditors: luochuan
 */

#include <iostream>
#include <unistd.h>

#include "agvmove.hpp"

using namespace std;

AGVMove::AGVMove()
{
	delta_x = 0;
	delta_y = 0;
	leftSpeed = 0;
	rightSpeed = 0;
}

void AGVMove::setVibrationError(float _x, float _y)
{
	delta_x = _x;
	delta_y = _y;
}

void AGVMove::checkStatus(float center1X, float center1Y, float center2X, float center2Y)
{
	AGV_STATUS status;
    /*
    	根据中心点坐标的变换，判断目标在小车前方的位置，向控制板发送指令
		To do    STOP, FORWARD, BACKWORD, LEFT, RIGHT, LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR
		设置一个小车振动误差范围 (delta(x), delta(y))，共有以下九种情况：
		1	若 (cx, cy) 在 (trackX +- delta(x), trackY +- delta(y)) 范围内，	则认为目标没动，小车处于停止状态
		2	若 cx 在 trackX +- delta(x) 范围内 && cy < trackY-delta(y)，		则认为目标在往前走，小车前进
		3	若 cx 在 trackX +- delta(x) 范围内 && cy > trackY+delta(y)，		则认为目标在往后走，小车后退
		4	若 cx < trackX-delta(x) && cy 在 trackY +- delta(y) 范围内，		则认为目标向左移动，小车左转
		5	若 cx > trackX+delta(x) && cy 在 trackY +- delta(y) 范围内，		则认为目标向右移动，小车右转
		6	若 cx < trackX-delta(x) && cy < trackY-delta(y)，					则认为目标向左前移动，小车左前运动
		7	若 cx > trackX+delta(x) && cy < trackY-delta(y)，					则认为目标向右前移动，小车右前运动
		8	若 cx < trackX-delta(x) && cy > trackY+delta(y)，					则认为目标向左后移动，小车左后运动
		9	若 cx > trackX-delta(x) && cy > trackY+delta(y)，					则认为目标向右后移动，小车右后运动
	*/
    
    if (center1X > center2X - delta_x && center1X < center2X + delta_x 
        && center1Y < center2Y - delta_y)
        status = AGV_FORWARD;

    else if (center1X > center2X - delta_x && center1X < center2X + delta_x 
        && center1Y > center2Y + delta_y)
        status = AGV_BACKWARD;

    else if (center1X < center2X - delta_x  
        && center1Y > center2Y - delta_y && center1Y < center2Y + delta_y)
        status = AGV_LEFT;

    else if (center1X > center2X + delta_x 
        && center1Y > center2Y - delta_y && center1Y < center2Y + delta_y)
        status = AGV_RIGHT;

    else if (center1X < center2X - delta_x 
        && center1Y < center2Y - delta_y)
        status = AGV_LEFT_FRONT;

    else if (center1X > center2X + delta_x 
        && center1Y < center2Y - delta_y)
        status = AGV_RIGHT_FRONT;

    else if (center1X < center2X - delta_x 
        && center1Y > center2Y + delta_y)
        status = AGV_LEFT_REAR;

    else if (center1X > center2X + delta_x 
        && center1Y < center2Y - delta_y)
        status = AGV_RIGHT_REAR;

	else
        status = AGV_STOP;

	move(status);
}

void AGVMove::setSpeed(int left, int right)
{
	leftSpeed += left;
	rightSpeed += right;
}

/*
	02 		09		01		01		00	64		01		00	64		d6
  起始符   数据长度  命令字  左轮方向   左轮转速    右轮方向   右轮转速     校验和
*/
void AGVMove::make_packet_and_send(int fd)
{
	char out[100];
	int temp = 0;

	out[0] = 0x02;										// 起始符 

	out[1] = 0x09;										// 数据长度

	out[2] = 0x01;										// 命令字

	out[3] = (leftSpeed >= 0) ? 1 : 0;					// 左轮方向
	temp = (leftSpeed >= 0) ? leftSpeed : 0 - leftSpeed;
	out[4] = temp / 256;								// 左轮速度
	out[5] = temp % 256;

	out[6] = (rightSpeed >= 0) ? 1 : 0;					// 右轮方向
	temp = (rightSpeed >= 0) ? rightSpeed : 0 - rightSpeed;
	out[7] = temp / 256;								// 右轮速度
	out[8] = temp % 256;

	for(int i = 1; i < 9; i++)							// 校验和
	{
		out[9] += out[i];
	}

	out[10] = 0;
	
	write(fd, out, 10);
}

void AGVMove::move(AGV_STATUS status)
{
    // 根据状态选择运动模式
    switch (status)
	{
		case AGV_STOP:
            goStop();
			cout << "指令收到！停止！" << endl;
			break;
				
		case AGV_FORWARD:
			goForward();
			cout << "指令收到！前进！" << endl;
    		break;

		case AGV_BACKWARD:
			goBackward();
			cout << "指令收到！后退！" << endl;
			break;

		case AGV_LEFT:
			goLeft();
			cout << "指令收到！左转！" << endl;
			break;

		case AGV_RIGHT:
			goRight();
			cout << "指令收到！右转！" << endl;
			break;

		case AGV_LEFT_FRONT:
			goLeftFront();
			cout << "指令收到！左前方行驶！" << endl;
			break;

		case AGV_RIGHT_FRONT:
			goRightFront();
			cout << "指令收到！右前方行驶！" << endl;
			break;

		case AGV_LEFT_REAR:
			goLeftRear();
			cout << "指令收到！左后方行驶！" << endl;
			break;

		case AGV_RIGHT_REAR:
			goRightRear();
			cout << "指令收到！右后方行驶！" << endl;
			break;

		default:
			cout << "指令错误！" << endl;
	}// switch
}

void AGVMove::goStop()
{
	
}

void AGVMove::goForward()
{
	setSpeed(10, 10);
}

void AGVMove::goBackward()
{

}

void AGVMove::goLeft()
{
	setSpeed(-10, 10);
}

void AGVMove::goRight()
{
	setSpeed(10, -10);
}

void AGVMove::goLeftFront()
{

}

void AGVMove::goRightFront()
{

}

void AGVMove::goLeftRear()
{

}

void AGVMove::goRightRear()
{

}
