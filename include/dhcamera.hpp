/*
 * @Description: 
 * @Author: luochuan
 * @Date: 2020-08-08 11:32:07
 * @LastEditTime: 2020-08-11 16:22:23
 * @LastEditors: luochuan
 */
#ifndef DHCAMERA_HPP
#define DHCAMERA_HPP

#include <opencv2/opencv.hpp>

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "CTimeCounter.hpp"

using namespace cv;

class DHCamera
{
private:
    int64_t g_nPixelFomat;          // 图像数据格式 GX_PIXEL_FORMAT_BAYER_BG8
    int64_t g_nColorFilter;         // bayer 插值的参数
    int32_t g_nImageSize;           // 图像的尺寸
    int32_t g_nImageWidth;          // 图像的宽度
    int32_t g_nImageHeight;         // 图像的高度
    
    bool g_bIsSnaping;              // 采集状态 开 or 关

    GX_FRAME_DATA g_frameData;      // 采集图像数据
    void* g_pRawImage;              // 将非8位raw数据转换成8位数据的时候的中转缓冲buffer
    void* g_pRgbImage;              // RGB 图像分配空间

    CTimeCounter g_objTimeCounter;  // 计时器 
    
public:
    /**
     * @description: 相机设备库的初始化，并打开相机设备
     * @param 构造函数 
     * @return {type} 
     */
    DHCamera();

    /**
     * @description: 
     * @param 析构函数
     * @return  
     */
    virtual ~DHCamera() {};

    /**
     * @description: 根据需求完成对相机的配置，并为采集和处理图像做准备
     * @param 初始化相机 
     * @return void 
     */
    void InitCamera();

    /**
     * @description: 初始化相机，发送采集指令
     * @param 开始 
     * @return void 
     */ 
    void StartMode();

    /**
     * @description: 停止采集，并关闭相机设备和设备库
     * @param 结束 
     * @return void
     */ 
    void EndMode();

    /**
     * @description: 获取图像 buffer，并将数据存储在 m_pRawImage
     * @param 
     * @return bool
     */
    bool GetImage();

    /**
     * @description: 获取图像，并将 RAW 格式图像转换为 RGB 图
     * @param 
     * @return Mat
     */ 
    Mat ProcessImage();

    /**
     * @description: 以 OpenCV 格式绘制图像
     * @param image 输出图像
     * @return void 
     */ 
    void ShowImage(const Mat& image);

    /**
     * @description: 显示相机设备信息
     * @param {type} 
     * @return void
     */
    void DisplayCameraInfo();
};

#endif
