#pragma once

#include <opencv2/opencv.hpp>

/**
 * @brief 处理单帧图像，检测装甲板灯条并进行PnP解算
 * @param frame 输入的BGR图像
 * @param binaryOut 输出的二值图
 * @param result 输出的检测结果图像（带标注）
 */
void processFrame(cv::Mat &frame, cv::Mat &binaryOut, cv::Mat &result);
