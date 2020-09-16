/*
 * @Description: 
 * @Author: luochuan
 * @Date: 2020-08-08 11:24:28
 * @LastEditTime: 2020-08-11 17:37:59
 * @LastEditors: luochuan
 */

#include <iostream>
#include <unistd.h>

#include "dhcamera.hpp"

using namespace std;

GX_DEV_HANDLE g_hDevice = NULL;	// 设备句柄指针

// 构造函数
DHCamera::DHCamera()
{
	GX_STATUS camStatus = GX_STATUS_SUCCESS;
	
	// 相机库初始化
	GXInitLib();
	if (camStatus == GX_STATUS_SUCCESS)
    {
        cout << "相机库初始化成功！" << endl;
    }
    else
    {
        cout << "相机库初始化失败！" << endl;
        return;
    }

	uint32_t nDeviceNum = 0;							// 枚举设备个数
	camStatus = GXUpdateDeviceList(&nDeviceNum, 1000);	// 更新设备个数
	if(camStatus != GX_STATUS_SUCCESS || nDeviceNum <= 0)
	{
		cout << "当前无可用设备!" << endl;
        return;
	}

	// 打开相机设备
	camStatus = GXOpenDeviceByIndex(1, &g_hDevice);
	if (camStatus == GX_STATUS_SUCCESS)
    {
        cout << "相机设备打开成功！" << endl;
    }
    else
    {
        cout << "相机设备打开失败！" << endl;
        return;
    }
}

// 根据需求完成对相机的配置，并为采集和处理图像做准备
void DHCamera::InitCamera()
{
	GX_STATUS camStatus = GX_STATUS_SUCCESS;
	int64_t nValue = 0;
	int64_t nPayLoadSize = 0;
 
	g_bIsSnaping = false;

	//设置采集模式为连续采集
	camStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

	// // 设置采集速度级别
	// GXSetInt(g_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 4);

	// 设置连续自动白平衡模式
	camStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

    // 设置连续自动曝光模式
	camStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
	
	// 获取图像数据格式(需在停止采集状态下设置)
	camStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_FORMAT, &g_nPixelFomat);

	// 获取相机输出数据的颜色格式(需在停止采集状态下设置)
	camStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_nColorFilter);

	// 获取图像的宽度(需在停止采集状态下设置)
	camStatus = GXGetInt(g_hDevice, GX_INT_WIDTH, &nValue);
	g_nImageWidth = (int32_t)nValue;

	// 获取图像的高度(需在停止采集状态下设置)
	camStatus = GXGetInt(g_hDevice, GX_INT_HEIGHT, &nValue);
	g_nImageHeight = (int32_t)nValue;
 
	// 获取图像数据大小(需在停止采集状态下设置)
	camStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
	g_nImageSize = nPayLoadSize;
	 
	do
	{
		// 根据获取的图像 buffer 大小 m_nImageSize 申请空间
		g_frameData.pImgBuf = malloc(nPayLoadSize);
		if (g_frameData.pImgBuf == nullptr)
		{
			camStatus = GX_STATUS_ERROR;
			break;
		}

		// // 为存储原始图像数据开辟空间
		// g_pRawImage = malloc(nPayLoadSize);
		// if (g_pRawImage == nullptr)
		// {
		// 	camStatus = GX_STATUS_ERROR;
		// 	break;
		// }

		// 为存储 RGB 图像数据开辟空间
        g_pRgbImage = malloc(nPayLoadSize * 3);
		if (g_pRgbImage == nullptr)
		{
			camStatus = GX_STATUS_ERROR;
			break;
		}
		
	} while (0);

	if (camStatus != GX_STATUS_SUCCESS)
	{
		if (g_frameData.pImgBuf != nullptr)
		{
			free(g_frameData.pImgBuf);
			g_frameData.pImgBuf = nullptr;
		}

		// if (g_pRawImage != nullptr)
		// {
		// 	free(g_pRawImage);
		// 	g_pRawImage = nullptr;
		// }
		
		if (g_pRgbImage != nullptr)
		{
			free(g_pRgbImage);
			g_pRgbImage = nullptr;
		}
	}
}

// 初始化相机，发送采集指令
void DHCamera::StartMode()
{
	GX_STATUS camStatus = GX_STATUS_SUCCESS;

	InitCamera();

	if(!g_bIsSnaping)
	{	
		// 发送开始采集命令
		camStatus = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);
		if (camStatus == GX_STATUS_SUCCESS)
            cout << "采集指令发送成功！" << endl;

		g_bIsSnaping = true;
	}
}

// 停止采集，并关闭相机设备和设备库
void DHCamera::EndMode()
{  
	GX_STATUS camStatus = GX_STATUS_SUCCESS;
	if(g_bIsSnaping)
	{
		// 发送停止采集命令
        camStatus = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);
		if (camStatus == GX_STATUS_SUCCESS)
            cout << "相机设备已停采！" << endl;
		
		g_bIsSnaping = false;
	}

	//释放 buffer 空间
	free(g_frameData.pImgBuf);
	g_frameData.pImgBuf = nullptr;
	
	// free(g_pRawImage);
	// g_pRawImage = nullptr;
		
	free(g_pRgbImage);
	g_pRgbImage = nullptr;

	// 关闭相机设备
	camStatus = GXCloseDevice(g_hDevice);
	if (camStatus == GX_STATUS_SUCCESS)
    {
        cout << "相机设备关闭成功！" << endl;
    }
    else
    {
        cout << "相机设备关闭失败！" << endl;
        return;
    } 
	
	// 关闭相机设备库
	camStatus = GXCloseLib();
	if (camStatus == GX_STATUS_SUCCESS)
    {
        cout << "相机库已关闭！" << endl;
    }
    else
    {
        cout << "相机库关闭失败！" << endl;
        return;
    } 
}

// 获取图像 buffer，并将数据存储在 m_pRawImage
bool DHCamera::GetImage()
{		
	if (g_nImageSize > 0)
    {
        // 调用 GXGetImage 取一帧图像
		GXGetImage(g_hDevice, &g_frameData, 100);
        if (g_frameData.nStatus == GX_FRAME_STATUS_SUCCESS)
        {
            cout << "图像获取成功！" << endl;
			return true;
		}
	}

	return false;
}

// 获取图像，并将 RAW 格式图像转换为 RGB 图
Mat DHCamera::ProcessImage()
{
	GX_STATUS camStatus = GX_STATUS_SUCCESS;

    // 对图像进行处理...
	g_objTimeCounter.Begin();
    camStatus = DxRaw8toRGB24(g_frameData.pImgBuf, g_pRgbImage, g_frameData.nWidth, g_frameData.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_nColorFilter), false);
	
	if (camStatus == GX_STATUS_SUCCESS)
    {
        cout << "图像格式由 RAW 转换成 RGB 格式！" << endl;
		cout << "DxRaw8toRGB24 耗时 ：" << g_objTimeCounter.End() << " ms" << endl;
    }

	// 转化为 OpenCV 格式便于处理和显示
	Mat image (Size(g_frameData.nWidth, g_frameData.nHeight), CV_8UC3, (void*)g_pRgbImage, Mat::AUTO_STEP);
	cvtColor(image, image, COLOR_BGR2RGB);

	return image;
}

// 以 OpenCV 格式绘制图像
void DHCamera::ShowImage(const Mat& image)
{
	namedWindow("Current", 0);
    imshow("Current", image);
	waitKey(30);
}

// 显示相机设备信息
void DHCamera::DisplayCameraInfo()
{
	cout << "图像尺寸：" << g_nImageWidth << " * " << g_nImageHeight << endl;
    cout << "图像数据格式：" << g_nPixelFomat << endl;
}