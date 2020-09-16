/*
 * @Description: KCF Tracker
 * @Author: Joao Faro, Christian Bailer, Joao F. Henriques
 * @Date: 2020-07-LI22 16:13:31
 * @LastEditTime: 2020-08-06 14:05:21
 * @LastEditors: luochuan
 */ 

/*
Constructor parameters, all boolean:
    hog: use HOG features (default), otherwise use raw pixels
    fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
    multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: horizontal area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame
 */



#pragma once

#include "tracker.hpp"

#ifndef _OPENCV_KCFTRACKER_HPP_
#define _OPENCV_KCFTRACKER_HPP_
#endif

class KCFTracker : public Tracker
{
public:
    // 构造函数
    KCFTracker(bool hog = true,             // 使用HOG特征
                bool fixed_window = true,   // 使用固定窗口大小
                bool multiscale = true,     // 使用多尺度
                bool lab = true);           // 使用Lab颜色空间特征

    // 初始化tracker
    // roi：初始目标框; image：跟踪的第一帧图像
    virtual void init(const cv::Rect &roi, cv::Mat image);
    
    // 更新基于新一帧的位置
    // image：新一帧图像
    virtual cv::Rect update(cv::Mat image);

    float lambda;                 // regularization                               正则化，0.0001
    float padding;                // extra area surrounding the target            目标对象周围的额外区域，2.5
    float output_sigma_factor;    // bandwidth of gaussian target                 目标对象的高斯带宽，随着 HOG/Lab 的选择而变化
    float interp_factor;          // linear interpolation factor for adaptation   （更新）自适应线性插值因子，随着 HOG/Lab 的选择而变化
    float sigma;                  // gaussian kernel bandwidth                    高斯卷积核带宽，随着 HOG/Lab 的选择而变化
    int cell_size;                // HOG cell size                                HOG图像块的大小，4
    int cell_sizeQ;               // cell size^2, to avoid repeated operations    HOG图像块包含的像素数，16，计算省事 
    int template_size;            // template size                                模板大小，在计算 _tmpl_sz 时，较大边长被归一成 96，而较小边长按比例缩小
    float scale_step;             // scale step for multi-scale estimation        多尺度估计的尺度步长
    float scale_weight;           // to downweight detection scores of            为了增加其他尺度检测时的稳定性，给检测结果峰值做一定衰减，为原来的0.95倍
                                  // other scales for added stability

protected:
    // 在当前帧检测目标
    // z：前一帧的训练/第一帧的初始化结果； x：当前帧当前尺度下的特征； peak_value：检测结果峰值
    cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

    // 使用当前图像的检测结果训练跟踪器
    // x：当前帧当前尺度下的特征； train_interp_factor：interp_factor
    void train(cv::Mat x, float train_interp_factor);

    // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN.
    // They must also be periodic (ie., pre-processed with a cosine window).
    // 使用带宽 sigma 计算高斯卷积核，用于输入图像X和Y之间的相对位移,必须都是 MxN 大小。
    // 二者必须通过一个余弦窗口预处理
    cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

    // Create Gaussian Peak. Function called only in the first frame.
    // 创建高斯峰矩阵，函数只在第一帧的时候执行
    cv::Mat createGaussianPeak(int sizey, int sizex);

    // Obtain sub-window from image, with replication-padding and extract features
    // 从图像得到子窗口，通过赋值填充并检测特征
    cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    // 初始化 hanning 汉宁窗口，函数只在第一帧被执行
    void createHanningMats();

    // Calculate sub-pixel peak for one dimension
    // 计算一维亚像素峰值
    float subPixelPeak(float left, float center, float right);

    cv::Mat _alphaf;          // 滤波器模型参数，用于检测部分中结果的计算
    cv::Mat _prob;            // 高斯回归标签，不再更改，用于训练
    cv::Mat _tmpl;            // 初始化模板，用于 detect 的 z
    cv::Mat _labCentroids;    // Lab质心数组

private:
    int size_patch[3];        // HOG 特征的 sizeY，sizeX，numFeatures
    cv::Mat hann;             // createHanningMats()的计算结果
    cv::Size _tmpl_sz;        // HOG图像块对应的数组大小
    float _scale;             // 修正成_tmpl_sz后的尺度大小
    int _gaussian_size;       //
    bool _hogfeatures;        // HOG标志位
    bool _labfeatures;        // Lab标志位
};
