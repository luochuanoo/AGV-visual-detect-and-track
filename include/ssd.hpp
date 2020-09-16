/*
 * @Description: 
 * @Author: luochuan
 * @Date: 2020-08-06 14:03:01
 * @LastEditTime: 2020-09-16 15:50:12
 * @LastEditors: luochuan
 */

#ifndef SSD_H
#define SSD_H

#include <iostream>
#include <cmath>

#include "torch/torch.h"
#include "torch/script.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std::chrono;
using namespace std;
using namespace cv;


class SSDetection
{
public:
    /**
     * @description: 构造函数
     * @param weight 模型权重 
     * @param device GPU or CPU
     * @return {type} 
     */
    SSDetection(const std::string& weight, torch::Device* device);

    /**
     * @description: 析构函数
     * @param {type} 
     * @return {type} 
     */
    virtual ~SSDetection() {};

    /**
     * @description: 前向传播计算图像数据
     * @param image 输入的图像数据
     * @return torch::Tensor
     */
    torch::Tensor Forward(const cv::Mat& image);

private:
    /**
     * @description: 设置基本参数
     * @param {type} 
     * @return void
     */
    void SetFixedParams();

    /**
     * @description: 加载模型权重和设备
     * @param weight 模型权重 
     * @param device GPU or CPU
     * @return void
     */
    void LoadTracedModule(const std::string& weight, torch::Device* device);

    /**
     * @description: 生成预选框
     * @param device GPU or CPU 
     * @return torch::Tensor 
     */
    torch::Tensor PriorBox (torch::Device device);

    /**
     * @description: 检测出目标位置
     * @param output 前向传播结果
     * @param prior_boxes 生成的预选框
     * @param nms_thresh NMS阈值 
     * @return torch::Tensor
     */
    torch::Tensor DetectionLayer(const torch::Tensor& output, const torch::Tensor& prior_boxes, float nms_thresh);

    /**
     * @description: 
     * @param {type} 
     * @return {type} 
     */
    static torch::Tensor Decoder(torch::Tensor loc, const torch::Tensor& priors);

    /**
     * @description: 非极大值抑制，用来快速搜索极大值，选择出最优位置
     * @param decode_loc 位置 
     * @param conf 置信度
     * @return torch::Tensor 
     */
    torch::Tensor nms(const torch::Tensor& decode_loc, const torch::Tensor& conf);

private:
    std::shared_ptr<torch::jit::script::Module> module_;    // 
    torch::Device* device_;                                 // 选择的设备类型 GPU or CPU

    int net_size_;                          // 网络输入尺寸
    vector<int> feature_maps_;              // 用于分类检测的尺度特征图
    vector<int> steps_;                     // 与网络输入尺寸之间的比例，恢复至网络输入尺寸大小需要的倍数
    vector<int> min_size_;                  // 预选框的最小尺寸
    vector<int> max_size_;                  // 预选框的最大尺寸
    vector<vector<int>> aspect_ratios_;     // 预选框的比例
    float nms_thresh_;                      // 非极大值抑制筛选阈值

};

#endif