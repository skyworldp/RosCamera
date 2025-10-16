#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class HikVideoPublisher : public rclcpp::Node
{
  public:
    HikVideoPublisher() : Node("hik_video_publisher")
    {
        // 视频路径和发布帧率在代码中设定（可按需修改）
        video_path_ = "/home/skyworld/文档/Ros/src/hik_video_publisher_cpp/resource/blue.mp4"; // 修改为实际视频文件路径
        publish_fps_ = 30.0; // 修改为所需的发布帧率

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cameraraw", 10);

        cap_.open(video_path_);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", video_path_.c_str());
            return;
        }

        // timer based on publish_fps_
        auto period = std::chrono::duration<double>(1.0 / publish_fps_);
        timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                         std::bind(&HikVideoPublisher::timerCallback, this));
    }

  private:
    void timerCallback()
    {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty())
        {
            // loop the video
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!cap_.read(frame) || frame.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Video read failed or empty frame");
                return;
            }
        }

        std_msgs::msg::Header hdr;
        hdr.stamp = this->now();
        hdr.frame_id = "camera_frame";

        auto img_msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
        pub_->publish(*img_msg);
    }

    std::string video_path_;
    double publish_fps_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikVideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
