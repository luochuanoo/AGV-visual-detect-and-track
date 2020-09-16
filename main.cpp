/*
 * @Descripttion: 图像读取 + 目标检测 + 目标跟踪
 * @version: 
 * @Author: LuoChuan
 * @Date: 2020-08-03 15:37:20
 * @LastEditors: luochuan
 * @LastEditTime: 2020-09-16 16:51:27
 */

#include <iostream>
#include <algorithm>
#include <termios.h>
#include <fcntl.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ssd.hpp"
#include "dhcamera.hpp"
//#include "ssd.hpp"
#include "kcftracker.hpp"
#include "agvmove.hpp"

using namespace std;
using namespace cv;

vector<string> VOC_CLASSES = {"background", "lowerbody"};

#define ERROR_X 0
#define ERROR_Y 0
const char* agvWheelPath = "/dev/agv_wheel";

/**
 * @description: 获取波特率
 * @param baudrate  波特率 
 * @return speed_t 
 */
static speed_t getBaudrate(int baudrate);

/**
 * @description: 
 * @param path
 * @param baudrate
 * @param flags
 * @return int 
 */
int open_tty(char* path, int baudrate, int flags);


int main(int argc, char* argv[])
{
	// int fd = open_tty(agvWheelPath, 9600, 0);

	// 将跟踪结果保存到 output.txt
	ofstream resultsFile;
	string resultsPath = "../data/output.txt";
	resultsFile.open(resultsPath);

/*****************************************************************************************************************
***************************************************** 目标检测 *****************************************************
******************************************************************************************************************/

	/************************************************ 搭建网络 ************************************************/

	torch::DeviceType device_type; 		// 设置 Device 类型

	// 判断是否支持 GPU 加速 torch::kCUDA or torch::kCPU
	if (torch::cuda::is_available())
	{
		cout << "CUDA available!" << endl;
		device_type = torch::kCUDA;
		cout << "Training on GPU." << endl;
	}
	else
	{
		device_type = torch::kCPU;
		cout << "Training on CPU." << endl;
	}

	torch::Device device(device_type);					// 定义设备类型 GPU or CPU
	string weight = "../data/ssd300_vgg_lowerbody.pt"; 	// 模型权重文件 TorchScript 格式
	SSDetection SSDnet(weight, &device);				// 搭建 SSD 训练网络

	/************************************************ 读取图像 ************************************************/

	DHCamera dhCam;
	dhCam.StartMode();			
	dhCam.DisplayCameraInfo();

	int preImgNums = 100;		// 避免相机开启阶段，图像不清楚问题
	while (!dhCam.GetImage() || preImgNums > 0)
	{
		cout << "图像获取失败，继续获取..." << endl;
		preImgNums--;
	}

	Mat frame = dhCam.ProcessImage();
	dhCam.ShowImage(frame);

	// imwrite("../data/frame.jpg", frame);

	/************************************************ 预测位置 ************************************************/

	// 网络前向传播计算
	torch::Tensor detect_result = SSDnet.Forward(frame);

	float width = frame.cols;	// 输入图像的宽
	float height = frame.rows;	// 输入图像的高

	// x1, y1, x2, y2, score, label
	// Eigen 库
	detect_result.select(1, 0).mul_(width);
	detect_result.select(1, 1).mul_(height);
	detect_result.select(1, 2).mul_(width);
	detect_result.select(1, 3).mul_(height);

	detect_result = detect_result.cpu();
	// Return a `TensorAccessor` for CPU `Tensor`s. You have to specify scalar type and
	auto detect_result_data = detect_result.accessor<float, 2>();

	float score = 0;	// 检测分数
	int numRoi = 0;		// 检测出的目标框的数量
	const int box_area_threshold = 25000;
	for (int i = 0; i < detect_result.size(0); i++)
	{
		score = detect_result_data[i][4];
		if (score > 0.5)
			numRoi++;
	}
	cout << "目标框的数量为" << numRoi << endl;

	Rect2d roi;		// 检测出的目标框

	for (int i = 0; i < detect_result.size(0); i++)
	{
		score = detect_result_data[i][4];						// 检测分数
		string label = VOC_CLASSES[detect_result_data[i][5]]; 	// 检测标签

		if (score > 0.5)
		{
			// 检测出的目标框
			roi.x = detect_result_data[i][0]; 									// 左上角坐标
			roi.y = detect_result_data[i][1];
			roi.width = detect_result_data[i][2] - detect_result_data[i][0]; 	// 宽和高
			roi.height = detect_result_data[i][3] - detect_result_data[i][1];
			
			if (box_area_threshold < roi.width * roi.height)
			{
				cout << "label: " << label << "    " << "score: " << score << endl;
				cv::rectangle(frame, cv::Point(detect_result_data[i][0], detect_result_data[i][1]), cv::Point(detect_result_data[i][2], detect_result_data[i][3]),
						  			cv::Scalar(0, 255, 0), 3, LINE_8);
				
				// 初始帧跟踪框中心
				float cx = roi.x + roi.width / 2.0f;
				float cy = roi.y + roi.height / 2.0f;
				resultsFile << roi.x << "\t" << roi.y << "\t" << roi.width << "\t" << roi.height << "\t" << cx << "\t" << cy << endl;
			}
		}
	}

	namedWindow("detect_result", 0);
	imshow("detect_result", frame);
	imwrite("detect_result.jpg", frame);

/*****************************************************************************************************************
***************************************************** 目标跟踪 *****************************************************
******************************************************************************************************************/

	/************************************************ 初始化 ************************************************/

	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE = true;
	bool LAB = true;

	// 创建 KCF 目标跟踪器
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	// 使用第一帧和目标框进行初始化
	tracker.init(roi, frame);						  		// 目标跟踪器初始化
	rectangle(frame, roi, Scalar(0, 255, 255), 3, LINE_8); 	// 绘制初始目标跟踪框（矩形）

	/************************************************ 开始跟踪 ************************************************/

	// AGVMove agvPlatform;
	// // 设置一个平台振动误差范围，减少因振动产生的影响
	// agvPlatform.setVibrationError(ERROR_X, ERROR_X);
	
	// 整幅图像中心点坐标
	float centerX = width / 2.0;
	float centerY = height / 2.0;
	// 待跟踪目标框中心点坐标
	float trackX = centerX;
	float trackY = roi.y + roi.height / 2;

	Rect track_result;		// 跟踪框

	cout << "开始跟踪过程，按 ESC 退出." << endl;
	while (true)
	{
		dhCam.GetImage();
		frame = dhCam.ProcessImage();

		if (!frame.empty())
		{
			// 更新当前帧
			track_result = tracker.update(frame);

			// 根据目标位置的更新，绘制每一帧目标跟踪框（矩形）
			rectangle(frame, Point(track_result.x, track_result.y), Point(track_result.x + track_result.width, track_result.y + track_result.height), 
						Scalar(0, 255, 255), 3, LINE_8);
			
			// 跟踪框中心
			float cx = track_result.x + track_result.width / 2.0f;
			float cy = track_result.y + track_result.height / 2.0f;

			// 输出目标跟踪框的位置
			// 左上角坐标 + 右下角坐标 + 中心点坐标
			resultsFile << track_result.x << "\t" << track_result.y << "\t" << track_result.width << "\t" << track_result.height << "\t" << cx << "\t" << cy << endl;

			imshow("Image", frame);
			
			/************************************************ AGV 运动 ************************************************/

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

			
			// agvPlatform.checkStatus(cx, cy, trackX, trackY);
			// agvPlatform.make_packet_and_send(fd);
			
			// waitKey(30);
		}

		// 按 ESC 键退出
		if (waitKey(1) == 27)
			break;
	}	// while

	return 0;
}

static speed_t getBaudrate(int baudrate)
{
	switch(baudrate)
	{
		case 0: return B0;
		case 50: return B50;
		case 75: return B75;
		case 110: return B110;
		case 134: return B134;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
		case 1800: return B1800;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 500000: return B500000;
		case 576000: return B576000;
		case 921600: return B921600;
		case 1000000: return B1000000;
		case 1152000: return B1152000;
		case 1500000: return B1500000;
		case 2000000: return B2000000;
		case 2500000: return B2500000;
		case 3000000: return B3000000;
		case 3500000: return B3500000;
		case 4000000: return B4000000;
		default: return -1;
	}
}

int open_tty(char* path, int baudrate, int flags)
{
	int fd;
	speed_t speed;

	if ((speed = getBaudrate(baudrate)) == -1)
		return -1;
	if ((fd = open(path, O_RDWR | flags)) == -1)
		return -1;

	struct termios cfg;
	if (tcgetattr(fd, &cfg))
	{
		close(fd);
		return -1;
	}
	cfmakeraw(&cfg);
	cfsetispeed(&cfg, speed);
	cfsetospeed(&cfg, speed);

	if (tcsetattr(fd, TCSANOW, &cfg))
	{
		close(fd);
		return -1;
	}
	
	printf("open %s ok,fd=%d\n", path, fd); 
	return fd;
}