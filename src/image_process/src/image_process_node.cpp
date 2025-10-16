#include "../include/ArmorMatcher.h"
#include "../include/process.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageProcessNode : public rclcpp::Node
{
  public:
    ImageProcessNode() : Node("image_process_node")
    {
        // 模型路径和参数在代码中设定（可按需修改）
        std::string model_path = "/home/skyworld/文档/Ros/src/image_process/resources/resnet_best_embedded.fixed.onnx";
        std::string labels_path = "/home/skyworld/文档/Ros/resources/labels.txt";
        show_windows_ = true; // 是否显示 OpenCV 窗口，改为 true 可查看实时处理

        // 加载 ArmorMatcher 模型（如果提供了路径）
        if (!model_path.empty())
        {
            auto matcher = std::make_shared<armor::ArmorMatcher>();
            bool loaded = false;

            if (!labels_path.empty())
            {
                loaded = matcher->loadWithLabels(model_path, labels_path);
                RCLCPP_INFO(this->get_logger(), "尝试加载模型: %s, 标签: %s", model_path.c_str(), labels_path.c_str());
            }
            else
            {
                loaded = matcher->load(model_path);
                RCLCPP_INFO(this->get_logger(), "尝试加载模型: %s", model_path.c_str());
            }

            if (loaded && matcher->isReady())
            {
                armor::setGlobalArmorMatcher(matcher);
                RCLCPP_INFO(this->get_logger(), "ArmorMatcher 模型加载成功");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "ArmorMatcher 模型加载失败: %s", matcher->lastError().c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "未提供模型路径，将跳过装甲板分类");
        }

        // 订阅 /cameraraw
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cameraraw", 10, std::bind(&ImageProcessNode::imageCallback, this, std::placeholders::_1));

        // 发布处理后的图像
        pub_result_ = this->create_publisher<sensor_msgs::msg::Image>("/image_result", 10);
        pub_binary_ = this->create_publisher<sensor_msgs::msg::Image>("/image_binary", 10);

        RCLCPP_INFO(this->get_logger(), "image_process_node 已启动，订阅 /cameraraw");
    }

  private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 将 ROS 图像消息转换为 OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat binary_out, result;

        // 调用 processFrame 进行装甲板检测（捕获所有异常以防止崩溃）
        try
        {
            processFrame(frame, binary_out, result);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "processFrame threw cv::Exception: %s",
                                  e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "processFrame threw exception: %s",
                                  e.what());
            return;
        }

        // 发布处理后的结果图像（仅在非空时）
        if (!result.empty())
        {
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = msg->header.frame_id;

            auto result_msg = cv_bridge::CvImage(header, "bgr8", result).toImageMsg();
            pub_result_->publish(*result_msg);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "processFrame returned empty result; skipping publish");
        }

        // 发布二值图（仅在非空时）
        if (!binary_out.empty())
        {
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = msg->header.frame_id;

            auto binary_msg = cv_bridge::CvImage(header, "mono8", binary_out).toImageMsg();
            pub_binary_->publish(*binary_msg);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "processFrame returned empty binary; skipping publish");
        }

        // 如果启用了窗口显示（仅显示非空图像）
        if (show_windows_)
        {
            if (!result.empty())
                cv::imshow("Result", result);
            if (!binary_out.empty())
                cv::imshow("Binary", binary_out);
            cv::waitKey(1);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_result_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_binary_;
    bool show_windows_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
