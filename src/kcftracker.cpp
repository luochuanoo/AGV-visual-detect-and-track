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
    padding: area surrounding the target, relative to its size
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

#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.hpp"
#include "ffttools.hpp"
#include "recttools.hpp"
#include "fhog.hpp"
#include "labdata.hpp"
#endif

// 初始化 KCF 类参数
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
{

    // Parameters equal in all cases
    // 恒等参数
    lambda = 0.0001;
    padding = 2.5; 
    output_sigma_factor = 0.125;

    if (hog)        // HOG
    {
        interp_factor = 0.012;
        sigma = 0.6; 
        cell_size = 4;
        _hogfeatures = true;

        if (lab)    // Lab
        {
            interp_factor = 0.005;
            sigma = 0.4; 
            output_sigma_factor = 0.1;

            _labfeatures = true;
            _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
            cell_sizeQ = cell_size*cell_size;
        }
        else
            _labfeatures = false;
    }
    else            // RAW
    {
        interp_factor = 0.075;
        sigma = 0.2; 
        cell_size = 1;
        _hogfeatures = false;

        if (lab)
        {
            printf("Lab features are only used with HOG features.\n");
            _labfeatures = false;
        }
    }

    if (multiscale)         // multiscale 多尺度
    {
        template_size = 96;

        scale_step = 1.05;
        scale_weight = 0.95;
        if (!fixed_window)
        {
            printf("Multiscale does not support non-fixed window.\n");
            fixed_window = true;
        }
    }
    else if (fixed_window)  // fit correction without multiscale
    {
        template_size = 96;
        scale_step = 1;
    }
    else
    {
        template_size = 1;
        scale_step = 1;
    }
}

// 使用第一帧及其目标跟踪框，初始化 KCF 跟踪器
// 得到 目标模板 _tmpl 和 _alpha 的值，用于预测目标在下一帧的位置
void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
{
    _roi = roi;                                                             // 目标框
    assert(roi.width >= 0 && roi.height >= 0);                              // 判断表达式的值，若为0，则先打印一条错误信息，然后调用 abort 来终止程序；若为1，则继续
    _tmpl = getFeatures(image, 1);                                          // 提取图像特征，初始化目标模板 _tmpl
    _prob = createGaussianPeak(size_patch[0], size_patch[1]);               // 高斯回归标签 _prob
    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));    // 滤波器模型参数 _alphaf，在 train 里面每帧修改
    train(_tmpl, 1.0);                                                      // 使用初始帧训练
 }

// 基于当前帧更新目标位置
cv::Rect KCFTracker::update(cv::Mat image)
{
    // 修正边界
    // 如果整个边界框全部在图片之外，对边界框进行调整，保证边界框与图片有一定的重合部分
    if (_roi.x + _roi.width <= 0)   _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0)  _roi.y = -_roi.height + 1;
    if (_roi.x >= image.cols - 1)   _roi.x = image.cols - 2;
    if (_roi.y >= image.rows - 1)   _roi.y = image.rows - 2;

    // 跟踪框中心
    float cx = _roi.x + _roi.width / 2.0f;
    float cy = _roi.y + _roi.height / 2.0f;

    // 尺度不变时检测峰值坐标
    float peak_value;
    cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);

    // 较大尺度和较小尺度进行检测
    // 如果存在多个尺度，选择计算得到的峰值最大的尺度作为目标框的尺度
    if (scale_step != 1)    // 多尺度 scale_step：1.05
    {
        // 较小尺度进行检测
        float new_peak_value;
        cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value);
        // 做减益 0.95，若峰值还比同尺度大就认为是目标
        if (scale_weight * new_peak_value > peak_value)
        {
            res = new_res;
            peak_value = new_peak_value;
            _scale /= scale_step;
            _roi.width /= scale_step;
            _roi.height /= scale_step;
        }

        // 较大尺度进行检测
        new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value);
        // 做减益 0.95，若峰值还比同尺度大就认为是目标
        if (scale_weight * new_peak_value > peak_value)
        {
            res = new_res;
            peak_value = new_peak_value;
            _scale *= scale_step;
            _roi.width *= scale_step;
            _roi.height *= scale_step;
        }
    }

    // Adjust by cell size and _scale
    // 使用中心坐标和尺度更新目标跟踪框
    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);     // 左上角坐标
    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);

    if (_roi.x >= image.cols - 1)   _roi.x = image.cols - 1;
    if (_roi.y >= image.rows - 1)   _roi.y = image.rows - 1;
    if (_roi.x + _roi.width <= 0)   _roi.x = -_roi.width + 2;
    if (_roi.y + _roi.height <= 0)  _roi.y = -_roi.height + 2;

    assert(_roi.width >= 0 && _roi.height >= 0);

    // 使用当前的检测框提取特征，并通过训练样本更新目标模板 _tmpl 和参数 _alpha
    cv::Mat x = getFeatures(image, 0);
    train(x, interp_factor);

    return _roi;    // 返回检测框
}

// Obtain sub-window from image, with replication-padding and extract features
// 通过填充从图像中获取子窗口，并提取特征，得到模板特征图 _tmpl
cv::Mat KCFTracker::getFeatures(const cv::Mat & image, bool inithann, float scale_adjust)
{
    cv::Rect extracted_roi; // 检测区域

    // roi 中心点位置坐标
    float cx = _roi.x + _roi.width / 2;
    float cy = _roi.y + _roi.height / 2;

    // inithann = 1
    if (inithann)
    {
        // 填充 roi 周围区域
        int padded_w = _roi.width * padding;
        int padded_h = _roi.height * padding;
        
        // 按照长宽比例来修改长宽大小，保证较长边为 template_size 的大小
        // Fit largest dimension to the given template size
        if (template_size > 1)
        {
            if (padded_w >= padded_h)
                _scale = padded_w / (float) template_size;  // fit to width
            else
                _scale = padded_h / (float) template_size;  //fit to height

            // 根据填充后的区域大小，反过来更新模板大小
            _tmpl_sz.width = padded_w / _scale;
            _tmpl_sz.height = padded_h / _scale;
        }
        else    // 如果没有提供模板尺寸，则使用填充后 roi 的大小为模板尺寸
        {
            _tmpl_sz.width = padded_w;
            _tmpl_sz.height = padded_h;
            _scale = 1;
            // original code from paper:
            /*if (sqrt(padded_w * padded_h) >= 100) {   //Normal size
                _tmpl_sz.width = padded_w;
                _tmpl_sz.height = padded_h;
                _scale = 1;
            }
            else {   //ROI is too big, track at half size
                _tmpl_sz.width = padded_w / 2;
                _tmpl_sz.height = padded_h / 2;
                _scale = 2;
            }*/
        }

        // 调整 _tmpl_sz 的长宽：偶数
        if (_hogfeatures)
        {
            // Round to cell size and also make it even
            _tmpl_sz.width = ( ( (int)(_tmpl_sz.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
            _tmpl_sz.height = ( ( (int)(_tmpl_sz.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
        }
        else    //Make number of pixels even (helps with some logic involving half-dimensions)
        {
            _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
            _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
        }
    }

    // 检测区域的大小
    extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
    extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;
    // 检测区域的左上角坐标
    extracted_roi.x = cx - extracted_roi.width / 2;
    extracted_roi.y = cy - extracted_roi.height / 2;

    // 提取目标区域像素，超边界则做填充
    cv::Mat FeaturesMap;                                                            // 特征图
    cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);   // 获取模板区域子图片
    
    // 按照比例放缩边界大小
    if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height)
        cv::resize(z, z, _tmpl_sz);  

    // 提取 HOG 特征点
    if (_hogfeatures)
    {
        IplImage z_ipl = z;
        CvLSVMFeatureMapCaskade *map;               // 申请指针
        getFeatureMaps(&z_ipl, cell_size, &map);    // 获取模板特征，给 map 赋值
        normalizeAndTruncate(map,0.2f);             // 归一化
        PCAFeatureMaps(map);
        size_patch[0] = map->sizeY;
        size_patch[1] = map->sizeX;
        size_patch[2] = map->numFeatures;

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();
        freeFeatureMapObject(&map);

        // 提取 Lab 特征
        if (_labfeatures)
        {
            cv::Mat imgLab;
            cvtColor(z, imgLab, CV_BGR2Lab);
            unsigned char *input = (unsigned char*)(imgLab.data);

            // Sparse output vector
            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0]*size_patch[1], CV_32F, float(0));

            int cntCell = 0;
            // Iterate through each cell
            for (int cY = cell_size; cY < z.rows-cell_size; cY+=cell_size)
            {
                for (int cX = cell_size; cX < z.cols-cell_size; cX+=cell_size)
                {
                    // Iterate through each pixel of cell (cX,cY)
                    for(int y = cY; y < cY+cell_size; ++y)
                    {
                        for(int x = cX; x < cX+cell_size; ++x)
                        {
                            // Lab components for each pixel
                            float l = (float)input[(z.cols * y + x) * 3];
                            float a = (float)input[(z.cols * y + x) * 3 + 1];
                            float b = (float)input[(z.cols * y + x) * 3 + 2];

                            // Iterate trough each centroid
                            float minDist = FLT_MAX;
                            int minIdx = 0;
                            float *inputCentroid = (float*)(_labCentroids.data);
                            for(int k = 0; k < _labCentroids.rows; ++k)
                            {
                                float dist = ( (l - inputCentroid[3*k]) * (l - inputCentroid[3*k]) )
                                           + ( (a - inputCentroid[3*k+1]) * (a - inputCentroid[3*k+1]) ) 
                                           + ( (b - inputCentroid[3*k+2]) * (b - inputCentroid[3*k+2]) );
                                if(dist < minDist)
                                {
                                    minDist = dist;
                                    minIdx = k;
                                }
                            }
                            // Store result at output
                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ; 
                            //((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ; 
                        }
                    }
                    cntCell++;
                }
            }
            // Update size_patch[2] and add features to FeaturesMap
            size_patch[2] += _labCentroids.rows;
            FeaturesMap.push_back(outputLab);
        }
    }
    else
    {
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch[0] = z.rows;
        size_patch[1] = z.cols;
        size_patch[2] = 1;  
    }
    
    if (inithann)
        createHanningMats();  // 初始化 hanning 窗，只在第一帧的时候执行一次  
    FeaturesMap = hann.mul(FeaturesMap);
    return FeaturesMap;
}

// 初始化 hanning 窗，只在第一帧的时候执行一次
void KCFTracker:: createHanningMats()
{   
    cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1],1), CV_32F, cv::Scalar(0));
    cv::Mat hann2t = cv::Mat(cv::Size(1,size_patch[0]), CV_32F, cv::Scalar(0)); 

    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float > (0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float > (i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

    cv::Mat hann2d = hann2t * hann1t;
    // HOG features
    if (_hogfeatures)
    {
        cv::Mat hann1d = hann2d.reshape(1,1); // Procedure do deal with cv::Mat multichannel bug
        
        hann = cv::Mat(cv::Size(size_patch[0]*size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
        for (int i = 0; i < size_patch[2]; i++)
        {
            for (int j = 0; j<size_patch[0]*size_patch[1]; j++)
            {
                hann.at<float>(i,j) = hann1d.at<float>(0,j);
            }
        }
    }
    // Gray features
    else
    {
        hann = hann2d;
    }
}

// 根据模板特征图，创建同样大小的高斯矩阵，只在第一帧的时候执行一次，用于计算 alpha 的值
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
{
    cv::Mat_<float> res(sizey, sizex);

    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
    float mult = -0.5 / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }
    return FFTTools::fftd(res);
}

// 根据特征图进行训练，更新当前帧的 _tmpl，_alpha
void KCFTracker::train(cv::Mat x, float train_interp_factor)
{
    using namespace FFTTools;

    cv::Mat k = gaussianCorrelation(x, x);
    cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));

    // 更新目标模板 _tmpl 和参数 _alpha
    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
    _alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf;
}

// 使用带宽 sigma 计算高斯核，用于计算输入图像X和Y之间的所有相对位移
// 必须都是 MxN 大小。二者必须都是周期的（通过一个 cos 窗口进行预处理）
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
{
    using namespace FFTTools;
    cv::Mat c = cv::Mat( cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0) );
    // HOG features
    if (_hogfeatures)
    {
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch[2]; i++)
        {
            x1aux = x1.row(i);                  // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch[0]);
            x2aux = x2.row(i).reshape(1, size_patch[0]);
            cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true); 
            caux = fftd(caux, true);
            rearrange(caux);
            caux.convertTo(caux,CV_32F);
            c = c + real(caux);
        }
    }
    // Gray features
    else
    {
        cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
        c = fftd(c, true);
        rearrange(c);
        c = real(c);
    }
    cv::Mat d; 
    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch[0]*size_patch[1]*size_patch[2]) , 0, d);

    cv::Mat k;
    cv::exp((-d / (sigma * sigma)), k);
    return k;
}

// 在当前帧检测目标
// z：前一帧样本;   x：当前帧;  peak_value：输出的峰值
cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value)
{
    using namespace FFTTools;

    // 做离散快速傅里叶变换得到计算结果 res
    cv::Mat k = gaussianCorrelation(x, z);                                      // 计算当前帧候选区域 x 和和是上一帧模板 z 之间的高斯核 k
    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));  // 相关响应

    // minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
    // 使用 opencv 的 minMaxLoc 来定位峰值坐标位置
    cv::Point2i pi;
    double pv;
    cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
    peak_value = (float) pv;

    //subpixel peak estimation, coordinates will be non-integer
    // 子像素峰值估计，坐标非整型
    cv::Point2f p((float)pi.x, (float)pi.y);

    if (pi.x > 0 && pi.x < res.cols-1)
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));

    if (pi.y > 0 && pi.y < res.rows-1)
        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));

    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;   // 返回峰值坐标
}

// Calculate sub-pixel peak for one dimension
// 使用幅值做差来定位峰值的位置，返回需要改变的偏移量的大小
float KCFTracker::subPixelPeak(float left, float center, float right)
{   
    float divisor = 2 * center - right - left;

    if (divisor == 0)
        return 0;
    
    return 0.5 * (right - left) / divisor;
}