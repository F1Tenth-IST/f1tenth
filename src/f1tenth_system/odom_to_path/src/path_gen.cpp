#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class OdomToPath : public rclcpp::Node {
public:
    OdomToPath() : Node("odom_to_path") {
        // Declaração dos parâmetros
        this->declare_parameter<std::string>("pose_topic", "");
        this->declare_parameter<std::string>("odom_topic", "");
        this->declare_parameter<double>("min_distance", 0.02);    // 2 cm
        this->declare_parameter<double>("publish_rate", 1.0);     // 1 Hz

        // Leitura dos parâmetros
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();

        //verifica se existem os dois tópicos
        if(odom_topic_ != "" && pose_topic_ != ""){
            RCLCPP_ERROR(this->get_logger(), "Por favor, defina apenas um dos tópicos: 'odom_topic' ou 'pose_topic'.");
            rclcpp::shutdown();
            return;
        }
        else if(odom_topic_ != ""){
            path_topic_ = odom_topic_ + "_path";
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&OdomToPath::odom_callback, this, std::placeholders::_1));
        }
        else if(pose_topic_ != ""){
            path_topic_ = pose_topic_ + "_path";
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10, std::bind(&OdomToPath::pose_callback, this, std::placeholders::_1));
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Por favor, defina um dos tópicos: 'odom_topic' ou 'pose_topic'.");
            rclcpp::shutdown();
            return;
        }

        min_distance_ = this->get_parameter("min_distance").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        // Subscrição e publicação
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

        // Timer configurado conforme publish_rate
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&OdomToPath::publish_path, this));

        RCLCPP_INFO(this->get_logger(),
            "Nó iniciado! Subscrito em '%s', publicando em '%s' (min_distance=%.3fm, rate=%.2fHz)",
            pose_topic_.c_str(), path_topic_.c_str(), min_distance_, publish_rate_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto &pose = msg->pose.pose.position;

        if (!has_last_pose_) {
            add_pose(msg);
            last_x_ = pose.x;
            last_y_ = pose.y;
            has_last_pose_ = true;
            return;
        }

        double dx = std::abs(pose.x - last_x_);
        double dy = std::abs(pose.y - last_y_);

        // Só adiciona se |dx| + |dy| > min_distance_
        if ((dx + dy) > min_distance_) {
            add_pose(msg);
            last_x_ = pose.x;
            last_y_ = pose.y;
        }
    }

    void add_pose(const nav_msgs::msg::Odometry::SharedPtr &msg) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;

        path_.header.stamp = msg->header.stamp;
        path_.header.frame_id = msg->header.frame_id;
        path_.poses.push_back(pose_stamped);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto &pose = msg->pose.position;

        if (!has_last_pose_) {
            path_.header.stamp = msg->header.stamp;
            path_.header.frame_id = msg->header.frame_id;
            path_.poses.push_back(*msg);
            last_x_ = pose.x;
            last_y_ = pose.y;
            has_last_pose_ = true;
            return;
        }

        double dx = std::abs(pose.x - last_x_);
        double dy = std::abs(pose.y - last_y_);

        // Só adiciona se |dx| + |dy| > min_distance_
        if ((dx + dy) > min_distance_) {
            path_.header.stamp = msg->header.stamp;
            path_.header.frame_id = msg->header.frame_id;
            path_.poses.push_back(*msg);
            last_x_ = pose.x;
            last_y_ = pose.y;
        }
    }

    void publish_path() {
        if (!path_.poses.empty()) {
            path_pub_->publish(path_);
        }
    }

    // Variáveis
    std::string pose_topic_, path_topic_, odom_topic_;
    double min_distance_;
    double publish_rate_;
    bool has_last_pose_ = false;
    double last_x_ = 0.0, last_y_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToPath>());
    rclcpp::shutdown();
    return 0;
}
