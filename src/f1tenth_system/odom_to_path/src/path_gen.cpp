#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OdomToPath : public rclcpp::Node {
public:
    OdomToPath() : Node("odom_to_path") {
        // Declaração de parâmetros com valores padrão
        this->declare_parameter<std::string>("odom_topic", "odom");

        // Obtendo os valores dos parâmetros
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        path_topic_ = odom_topic_ + "_path";

        // Criando subscrição e publicação
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&OdomToPath::odom_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Nó iniciado! Subscrito em '%s', publicando em '%s'",
                    odom_topic_.c_str(), path_topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;

        path_.header.stamp = msg->header.stamp;
        path_.header.frame_id = msg->header.frame_id;
        path_.poses.push_back(pose_stamped);

        path_pub_->publish(path_);
    }

    // Variáveis
    std::string odom_topic_, path_topic_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToPath>());
    rclcpp::shutdown();
    return 0;
}
