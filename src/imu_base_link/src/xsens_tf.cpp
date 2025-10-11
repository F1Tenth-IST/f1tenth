#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Dense>
#include <string>


class ImuTfConverter : public rclcpp::Node
{
public:
    ImuTfConverter(const std::string & imu_topic)
        : Node("imu_tf_converter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
          imu_topic_(imu_topic), imu_frame_("")
    {

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10,
            std::bind(&ImuTfConverter::imuCallback, this, std::placeholders::_1));        

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_base_link"+imu_topic_, 10);

        timer_print_status_ = this->create_wall_timer(
            std::chrono::seconds(5), // Print status every second
            [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Received Xsens Msg: %.2f hz",
                            rcv_xsens_count_/5.0);
                rcv_xsens_count_ = 0;
            });

    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (imu_frame_.empty()) {
            imu_frame_ = msg->header.frame_id;
            RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s (frame: %s)",
                    imu_topic_.c_str(), imu_frame_.c_str());
            get_imu_tf();
        }
        rcv_xsens_count_++;

        // 1. Transformar orientação
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(msg->orientation, q_orig);
        
        q_new = q_orig * imu_rot_.inverse();
        q_new.normalize();

        // 2. Transformar vetores (acc e vel angular)
        geometry_msgs::msg::Vector3Stamped acc_in, acc_out, gyro_in, gyro_out;

        acc_in.vector = msg->linear_acceleration;
        acc_in.header = msg->header;
        gyro_in.vector = msg->angular_velocity;
        gyro_in.header = msg->header;

        try {
            tf_buffer_.transform(acc_in, acc_out, "base_link");
            tf_buffer_.transform(gyro_in, gyro_out, "base_link");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Vector transform failed: %s", ex.what());
            return;
        }

        // 3. Criar nova mensagem
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = msg->header.stamp;
        imu_msg.header.frame_id = "base_link";
        imu_msg.orientation = tf2::toMsg(q_new);
        imu_msg.linear_acceleration = acc_out.vector;
        imu_msg.angular_velocity = gyro_out.vector;

        // Copiar covariâncias
        // Covariâncias - transformar
        imu_msg.orientation_covariance = rotateCovariance(msg->orientation_covariance, imu_rot_);
        imu_msg.angular_velocity_covariance = rotateCovariance(msg->angular_velocity_covariance, imu_rot_);
        imu_msg.linear_acceleration_covariance = rotateCovariance(msg->linear_acceleration_covariance, imu_rot_);


        imu_pub_->publish(imu_msg);
    }


    std::array<double, 9> rotateCovariance(const std::array<double, 9>& cov_in, const tf2::Quaternion& q)
    {
        Eigen::Matrix3d R = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
        Eigen::Map<const Eigen::Matrix3d> cov_orig(cov_in.data());
        Eigen::Matrix3d cov_rotated = R * cov_orig * R.transpose();

        std::array<double, 9> cov_out;
        Eigen::Map<Eigen::Matrix3d>(cov_out.data()) = cov_rotated;
        return cov_out;
    }

    void get_imu_tf()
    {
        geometry_msgs::msg::TransformStamped transform;
        // Get the transform from the configured IMU frame to base_link
        rclcpp::Rate rate(1.0); // 1 Hz
        while (rclcpp::ok()) {
            try {
                transform = tf_buffer_.lookupTransform("base_link", imu_frame_, tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "Transform found: %s to base_link", imu_frame_.c_str());
                RCLCPP_INFO(this->get_logger(), "Translation: [%.3f, %.3f, %.3f]",
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z);
                // Log roll, pitch, yaw (radians) computed from the quaternion
                tf2::Quaternion q;
                tf2::fromMsg(transform.transform.rotation, q);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                RCLCPP_INFO(this->get_logger(), "Rotation (RPY, rad): [%.3f, %.3f, %.3f]", roll, pitch, yaw);

                tf2::fromMsg(transform.transform.rotation, imu_rot_);
                break;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Waiting for %s->base_link transform: %s",
                            imu_frame_.c_str(), ex.what());
                rate.sleep();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Transform from %s to base_link obtained.", imu_frame_.c_str());
    }


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_print_status_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    tf2::Quaternion imu_rot_; // Transform from IMU frame to base_link

    int  rcv_xsens_count_ = 0;
    std::string imu_topic_;
    std::string imu_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string imu_topic = "/imu/data";
    if (argc > 1) {
        imu_topic = argv[1];
    }

    rclcpp::spin(std::make_shared<ImuTfConverter>(imu_topic));
    rclcpp::shutdown();
    return 0;
}
