// ...existing code...
#include "PixelType.h"
#include "cameral_func/HikCamera.h"                     // 包含海康相机头文件
#include "cv_bridge/cv_bridge.h"                        //Mat与Image消息转换
#include "rcl_interfaces/msg/set_parameters_result.hpp" //参数设置结果
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp" // 图像消息类型
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;

class HikCameraNode : public rclcpp::Node
{
  public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("hik_camera_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "HikCameraNode 已构建");
        constexpr double default_frame_rate = 20.0;
        constexpr unsigned int default_pixel_format = PixelType_Gvsp_BGR8_Packed;
        // 声明参数并设置默认值
        this->declare_parameter<double>("exposure_time", 2000.0);
        this->declare_parameter<double>("gain", 7.0);
        this->declare_parameter<bool>("trigger_mode", false);
        this->declare_parameter<double>("frame_rate", default_frame_rate); // 帧率 (Hz)
        this->declare_parameter<int>("pixel_format",
                                     static_cast<int>(default_pixel_format)); // 像素格式（SDK 数值编码）
        this->declare_parameter<std::string>("frame_id", frame_id_);

        // 参数数据
        exposure_time_ = this->get_parameter("exposure_time").as_double();
        gain_ = this->get_parameter("gain").as_double();
        trigger_mode_ = this->get_parameter("trigger_mode").as_bool();
        frame_rate_ = this->get_parameter("frame_rate").as_double();
        pixel_format_ = static_cast<unsigned int>(this->get_parameter("pixel_format").as_int());
        frame_id_ = this->get_parameter("frame_id").as_string();
        RCLCPP_INFO(this->get_logger(), "等待相机连接，on_publish 定时器将尝试自动重连");
        last_open_attempt_valid_ = false;

        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                std::ostringstream reason;

                auto append_reason = [&](const std::string &text) {
                    if (!reason.str().empty())
                    {
                        reason << "; ";
                    }
                    reason << text;
                };

                auto fail = [&](const std::string &name) {
                    result.successful = false;
                    append_reason(name + " 设置失败: " + cam.GetLastError());
                };

                for (const auto &p : params)
                {
                    const std::string &param_name = p.get_name();
                    if (param_name == "exposure_time")
                    {
                        double v = p.as_double();
                        double previous = exposure_time_;
                        if (!cam.SetExposureTime(static_cast<float>(v)))
                        {
                            fail("曝光时间");
                            exposure_time_ = previous;
                        }
                        else
                        {
                            exposure_time_ = v;
                        }
                    }
                    else if (param_name == "gain")
                    {
                        double v = p.as_double();
                        double previous = gain_;
                        if (!cam.SetGain(static_cast<float>(v)))
                        {
                            fail("增益");
                            gain_ = previous;
                        }
                        else
                        {
                            gain_ = v;
                        }
                    }
                    else if (param_name == "trigger_mode")
                    {
                        bool v = p.as_bool();
                        bool previous = trigger_mode_;
                        if (!cam.SetTriggerMode(v))
                        {
                            fail("触发模式");
                            trigger_mode_ = previous;
                        }
                        else
                        {
                            trigger_mode_ = v;
                        }
                    }
                    else if (param_name == "frame_rate")
                    {
                        double v = p.as_double();
                        if (v <= 0.0)
                        {
                            result.successful = false;
                            append_reason("帧率必须大于 0");
                            continue;
                        }
                        double previous = frame_rate_;
                        if (!cam.SetFrameRate(static_cast<float>(v)))
                        {
                            fail("帧率");
                            frame_rate_ = previous;
                            continue;
                        }
                        frame_rate_ = v;
                    }
                    else if (param_name == "pixel_format")
                    {
                        int v = p.as_int();
                        unsigned int requested = static_cast<unsigned int>(v);
                        unsigned int previous = pixel_format_;
                        if (!cam.SetPixelFormat(requested))
                        {
                            fail("像素格式");
                            pixel_format_ = previous;
                            continue;
                        }
                        pixel_format_ = requested;
                    }
                    else if (param_name == "frame_id")
                    {
                        frame_id_ = p.as_string();
                    }
                }

                result.reason = reason.str();
                return result;
            });
        // 初始化 FPS 统计基准时间
        last_fps_time_ = this->now();

        // 创建 publisher：发布相机图像
        // rclcpp::QoS qos(rclcpp::KeepLast(5));
        // qos.reliable();
        // qos.transient_local();
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("cameraraw", 10);
        // 使用固定 100 Hz 的定时器来轮询并发布（帧率由相机内部控制）
        constexpr double fixed_hz = 100.0;
        auto period = std::chrono::duration<double>(1.0 / fixed_hz);
        timer_ = this->create_wall_timer(period, std::bind(&HikCameraNode::on_publish, this));
    }

    ~HikCameraNode()
    {
        RCLCPP_INFO(this->get_logger(), "HikCameraNode 正 在析构");
        // 在此处释放摄像头资源
        cam.Close();
    }

  private:
    void on_publish()
    {

        const auto throttle_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(2s).count();
        // 确保采集正在进行
        if (!cam.IsGrabbing())
        {
            if (!cam.StartGrabbing())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ns, "重新开始采集失败: %s",
                                     cam.GetLastError().c_str());
                return;
            }
        }

        hik::ImageData imageData;
        if (!cam.GrabImage(imageData, 1000, pixel_format_))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ns, "获取图像失败: %s",
                                 cam.GetLastError().c_str());
            return;
        }

        if (imageData.data == nullptr || imageData.width == 0 || imageData.height == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ns, "收到无效图像帧");
            return;
        }

        std::string encoding = pixelFormatToEncoding(imageData.pixelFormat);
        int cvType = CV_8UC3;
        if (encoding == sensor_msgs::image_encodings::MONO8)
        {
            cvType = CV_8UC1;
        }
        else if (encoding == sensor_msgs::image_encodings::RGB8 || encoding == sensor_msgs::image_encodings::BGR8)
        {
            cvType = CV_8UC3;
        }
        else if (encoding == sensor_msgs::image_encodings::RGBA8 || encoding == sensor_msgs::image_encodings::BGRA8)
        {
            cvType = CV_8UC4;
        }
        else
        {
            cvType = CV_8UC3;
            encoding = sensor_msgs::image_encodings::BGR8;
        }

        cv::Mat frame(imageData.height, imageData.width, cvType, imageData.data);

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = frame_id_;

        auto msg = cv_bridge::CvImage(header, encoding, frame).toImageMsg();
        pub_->publish(*msg);

        // 成功发布后统计 FPS（基于发布次数，而非订阅者消费）
        ++publish_count_;
        auto now = this->now();
        auto elapsed = now - last_fps_time_;
        if (elapsed.seconds() >= 1.0)
        {
            unsigned int count = publish_count_.load();
            double fps = static_cast<double>(count) / elapsed.seconds();
            RCLCPP_INFO(this->get_logger(), "publish fps: %.2f (count=%u, elapsed=%.2fs)", fps, count,
                        elapsed.seconds());
            // reset
            publish_count_.store(0);
            last_fps_time_ = now;
        }
    }

    std::string pixelFormatToEncoding(unsigned int pixel_format) const
    {
        switch (pixel_format)
        {
        case PixelType_Gvsp_Mono8:
            return sensor_msgs::image_encodings::MONO8;
        case PixelType_Gvsp_RGB8_Packed:
            return sensor_msgs::image_encodings::RGB8;
        case PixelType_Gvsp_BGR8_Packed:
            return sensor_msgs::image_encodings::BGR8;
        case PixelType_Gvsp_RGBA8_Packed:
            return sensor_msgs::image_encodings::RGBA8;
        case PixelType_Gvsp_BGRA8_Packed:
            return sensor_msgs::image_encodings::BGRA8;
        default:
            return sensor_msgs::image_encodings::BGR8;
        }
    }

    //相机实例、publisher 与定时器
    hik::HikCamera cam;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double frame_rate_ = 30.0;
    unsigned int pixel_format_ = PixelType_Gvsp_BGR8_Packed;
    std::string frame_id_ = "hik_camera_optical_frame";
    std::atomic_uint publish_count_{0};
    rclcpp::Time last_fps_time_;
    //参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// main 保持简单：初始化 ROS2 并创建类节点实例
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikCameraNode>();
    RCLCPP_INFO(node->get_logger(), "节点已初始化: %s", node->get_name());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// ...existing code...