// ROS
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <exception>
#include <image_transport/image_transport.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// Daheng
#include "GxCamera.hpp"

namespace daheng_camera {
class AcquisitionNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new Acquisition Node object
     *
     * @param options
     */
    explicit AcquisitionNode(const rclcpp::NodeOptions &options)
        : Node("daheng_camera", options) {
        RCLCPP_INFO(get_logger(), "acquisition node start");
        // set log level
        // this->declare_parameter("rcl_log_level", 0);
        // this->get_parameter("rcl_log_level", log_level_);
        // this->get_logger().set_level((rclcpp::Logger::Level)log_level_);
        // param init
        camera_name_ = declare_parameter("camera_name", "daheng1");
        using_video0_ = declare_parameter("using_video0", false);
        bool use_sensor_data_qos =
            declare_parameter("use_sensor_data_qos", false);
        exposure_time_ = declare_parameter("exposure_time", 6000);
        gain_ = declare_parameter("gain", 12.0);

        // camera init and open
        if (!using_video0_) {
            size_t open_false_count = 0;
            bool camera_init_ok = camera_.initial(exposure_time_, gain_) &&
                                  camera_.openDevice(false, "SN");
            while (!camera_init_ok && rclcpp::ok()) {
                ++open_false_count;
                RCLCPP_WARN(get_logger(), "open device failed %lu times!",
                            open_false_count);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                camera_init_ok = camera_.initial(exposure_time_, gain_) &&
                                 camera_.openDevice(false, "SN");
            }
        } else {
            video0_camera_ = cv::VideoCapture(0);
        }
        if (!rclcpp::ok()) {
            exit(0);
        }
        RCLCPP_INFO(get_logger(), "open device success");

        // pub init
        auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                       : rmw_qos_profile_default;
        camera_pub_ =
            image_transport::create_camera_publisher(this, "image_raw", qos);
        // image_pub_ = image_transport::create_publisher(this, "/image_raw",
        // qos); cam_info_sub_ =
        // this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info",
        // rclcpp::SensorDataQoS());
        camera_info_manager_ =
            std::make_unique<camera_info_manager::CameraInfoManager>(
                this, camera_name_);
        auto camera_info_url = declare_parameter(
            "camera_info_url",
            //"package://auto_aim_bringup/config/camera_info1.yaml");
            "package://daheng_camera/config/camera_info1.yaml");
        if (camera_info_manager_->validateURL(camera_info_url)) {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            camera_info_msg_ = camera_info_manager_->getCameraInfo();
            RCLCPP_INFO(get_logger(), "camera info load %s success",
                        camera_info_url.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s",
                        camera_info_url.c_str());
        }
        RCLCPP_INFO(get_logger(), "Publishing image and camera info!");
        camera_info_msg_.header.frame_id = "camera_optical_frame";
        image_msg_.header.frame_id = "camera_optical_frame";
        image_msg_.encoding = "bgr8";
        // timer binding callback function
        timer_ =
            create_wall_timer(std::chrono::milliseconds(0),
                              std::bind(&AcquisitionNode::timerCallBack, this));
    }

  private:
    // int log_level_;
    GxCamera camera_;
    cv::VideoCapture video0_camera_;
    bool using_video0_;
    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;
    // image_transport::Publisher image_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    // params
    std::string camera_name_;
    size_t exposure_time_;
    double gain_;
    std::unique_ptr<camera_info_manager::CameraInfoManager>
        camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    void reopenCamera();
    /**
     * @brief callback function for publishing message
     *
     */
    void timerCallBack();
};
void AcquisitionNode::timerCallBack() {
    cv::Mat src;
    float timestamp;
    if (!using_video0_) {
        while (!camera_.read(&src, &timestamp))
            reopenCamera();
    } else {
        video0_camera_.read(src);
    }
    camera_info_msg_.header.stamp = image_msg_.header.stamp = now();
    // image_msg_.height = 1280;
    // image_msg_.width = 1024;
    // image_msg_.step = 1024 * 3;
    // image_msg_.data.resize(1280 * 1024 * 3);
    // cv::imencode(".jpg", src, image_msg_.data);
    cv_bridge::CvImage cv_image(image_msg_.header, image_msg_.encoding, src);
    cv_image.toImageMsg(image_msg_);
    // cam_info_sub_->publish(camera_info_msg_);
    // image_pub_.publish(image_msg_);
    camera_pub_.publish(image_msg_, camera_info_msg_);
}

void AcquisitionNode::reopenCamera() {
    RCLCPP_WARN(get_logger(), "Attempting to restart!");
    size_t open_false_count = 0;
    camera_.close();
    bool camera_init_ok = camera_.initial(exposure_time_, gain_) &&
                          camera_.openDevice(false, "SN");
    while (!camera_init_ok && rclcpp::ok()) {
        open_false_count++;
        RCLCPP_WARN(get_logger(), "reopen device failed %lu times!",
                    open_false_count);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        camera_.close();
        camera_init_ok = camera_.initial(exposure_time_, gain_) &&
                         camera_.openDevice(false, "SN");
    }
    RCLCPP_INFO(get_logger(), "reopen camera success!");
}

} // namespace daheng_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(daheng_camera::AcquisitionNode)
