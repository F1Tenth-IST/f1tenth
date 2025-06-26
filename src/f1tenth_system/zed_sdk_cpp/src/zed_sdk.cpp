#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/opencv.hpp>

class ZedRGBPublisher : public rclcpp::Node
{
public:
    ZedRGBPublisher() : Node("zed_sdk_rgb_publisher")
    {
        this->declare_parameter("fps", 30);
        this->declare_parameter("resolution", "HD720");
        this->declare_parameter("imu_frequency", 50);

        int fps = this->get_parameter("fps").as_int();
        std::string resolution_str = this->get_parameter("resolution").as_string();
        int imu_frequency = this->get_parameter("imu_frequency").as_int();

        RCLCPP_INFO(this->get_logger(), "Camera FPS: %d", fps);
        RCLCPP_INFO(this->get_logger(), "Camera Resolution: %s", resolution_str.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU Frequency: %d", imu_frequency);
        // Set resolution
        sl::RESOLUTION res = sl::RESOLUTION::HD720;
        if (resolution_str == "HD1080")
            res = sl::RESOLUTION::HD1080;
        else if (resolution_str == "HD2K")
            res = sl::RESOLUTION::HD2K;
        else if (resolution_str == "VGA")
            res = sl::RESOLUTION::VGA;

        // ZED Init parameters
        sl::InitParameters init_params;
        init_params.camera_resolution = res;
        init_params.camera_fps = fps;
        init_params.coordinate_units = sl::UNIT::MILLIMETER;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.sdk_verbose = true;
        init_params.camera_image_flip = sl::FLIP_MODE::OFF;
        init_params.camera_disable_self_calib = true;
        init_params.sdk_gpu_id = 0;

        // Open ZED camera
        zed_ = std::make_unique<sl::Camera>();
        sl::ERROR_CODE err = zed_->open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ZED: %s", sl::toString(err).c_str());
            rclcpp::shutdown();
            return;
        }

        auto info = zed_->getCameraInformation();
        sl::SensorParameters &sensor_parameters = info.sensors_configuration.accelerometer_parameters;
        RCLCPP_INFO(this->get_logger(), "Sensor Type: %d", static_cast<int>(sensor_parameters.type));
        RCLCPP_INFO(this->get_logger(), "Sampling Rate: %f", sensor_parameters.sampling_rate);
        RCLCPP_INFO(this->get_logger(), "Range: %f", sensor_parameters.range);
        RCLCPP_INFO(this->get_logger(), "Resolution: %f", sensor_parameters.resolution);

        // ROS 2 publishers
        publisher_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw", 10);
        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("zed2i/imu/data", 10);

        // Timer
        // Launch image publishing in a separate thread
        std::thread([this, fps]() {
            rclcpp::Rate rate(fps);
            while (rclcpp::ok()) {
            this->pub_image();
            rate.sleep();
            }
        }).detach();

        // Launch IMU publishing in a separate thread
        std::thread([this, imu_frequency]() {
            rclcpp::Rate rate(imu_frequency);
            while (rclcpp::ok()) {
            this->pub_imu();
            rate.sleep();
            }
        }).detach();

        // Create threads for image and IMU publishing
        //std::thread image_thread(&ZedRGBPublisher::pub_image, this);
        //std::thread imu_thread(&ZedRGBPublisher::pub_imu, this);

        // Detach threads so they run independently
        //image_thread.detach();
        //imu_thread.detach();
        // Start publishing
        // pub_imu();
    }

private:
    void pub_image()
    {
        if(zed_->grab() == sl::ERROR_CODE::SUCCESS)
        {
            sl::Mat zed_image;
            zed_->retrieveImage(zed_image, sl::VIEW::SIDE_BY_SIDE, sl::MEM::CPU); // CPU memory

            sensor_msgs::msg::Image msg;
            msg.header.stamp = this->now();
            msg.height = zed_image.getHeight();
            msg.width = zed_image.getWidth();
            msg.encoding = "bgra8";
            msg.is_bigendian = 0;
            msg.step = zed_image.getStepBytes(sl::MEM::CPU);
            msg.data.assign(zed_image.getPtr<sl::uchar1>(sl::MEM::CPU), zed_image.getPtr<sl::uchar1>(sl::MEM::CPU) + (msg.step * msg.height));

            publisher_rgb_->publish(msg);
        }
    }

    void pub_imu()
    {
        sl::SensorsData sensors_data;

        if (zed_->getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get IMU data");
            return;
        }

        // Publish IMU data
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "zed2i";
        msg.linear_acceleration.x = sensors_data.imu.linear_acceleration.x;
        msg.linear_acceleration.y = sensors_data.imu.linear_acceleration.y;
        msg.linear_acceleration.z = sensors_data.imu.linear_acceleration.z;

        matrix3fToArray(sensors_data.imu.linear_acceleration_covariance, msg.linear_acceleration_covariance);

        msg.angular_velocity.x = sensors_data.imu.angular_velocity.x;
        msg.angular_velocity.y = sensors_data.imu.angular_velocity.y;
        msg.angular_velocity.z = sensors_data.imu.angular_velocity.z;

        matrix3fToArray(sensors_data.imu.angular_velocity_covariance, msg.angular_velocity_covariance);

        msg.orientation.x = sensors_data.imu.pose.getOrientation().ox;
        msg.orientation.y = sensors_data.imu.pose.getOrientation().oy;
        msg.orientation.z = sensors_data.imu.pose.getOrientation().oz;
        msg.orientation.w = sensors_data.imu.pose.getOrientation().ow;

        publisher_imu_->publish(msg);
    }

    // Converts sl::Matrix3f to std::array<double, 9>
    void
    matrix3fToArray(const sl::Matrix3f &mat, std::array<double, 9> &arr)
    {
        arr[0] = static_cast<double>(mat.r00);
        arr[1] = static_cast<double>(mat.r01);
        arr[2] = static_cast<double>(mat.r02);
        arr[3] = static_cast<double>(mat.r10);
        arr[4] = static_cast<double>(mat.r11);
        arr[5] = static_cast<double>(mat.r12);
        arr[6] = static_cast<double>(mat.r20);
        arr[7] = static_cast<double>(mat.r21);
        arr[8] = static_cast<double>(mat.r22);
    }

    std::unique_ptr<sl::Camera> zed_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;

    struct TimestampHandler
    {

        // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
        inline bool isNew(sl::Timestamp &ts_curr, sl::Timestamp &ts_ref)
        {
            bool new_ = ts_curr > ts_ref;
            if (new_)
                ts_ref = ts_curr;
            return new_;
        }
        // Specific function for IMUData.
        inline bool isNew(sl::SensorsData::IMUData &imu_data)
        {
            return isNew(imu_data.timestamp, ts_imu);
        }
        // Specific function for MagnetometerData.
        inline bool isNew(sl::SensorsData::MagnetometerData &mag_data)
        {
            return isNew(mag_data.timestamp, ts_mag);
        }
        // Specific function for BarometerData.
        inline bool isNew(sl::SensorsData::BarometerData &baro_data)
        {
            return isNew(baro_data.timestamp, ts_baro);
        }

        sl::Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedRGBPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
