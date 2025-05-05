#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // para tf2::getYaw()
#include <tf2/utils.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h> // Add this include for tf2::Quaternion
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <cmath>
#include <vector>

// Include ACADOS headers
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_mpc_model.h"
#include "acados/utils/print.h"
#include "acados/utils/math.h"

// blasfeo
#include "blasfeo_d_aux_ext_dep.h"

#define N MPC_MODEL_N // Prediction horizon


class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        // Publishers and Subscribers
        control_vesc_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
        // state_sub_ = nh.subscribe("current_state", 10, &MPCNode::stateCallback, this);
        //  Update subscriptions to ROS 2 style
        solved_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("mpc/solved_time", 10);
        ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_path", 10);
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        reference_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc/ref_traj", qos_settings);
        state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/state_array", 10);
        control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/control_array", 10);

        control_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/control_vector", 10);
        state_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("mpc/state_vector", 10);

        odomm_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));

        vesc_servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/vesc/servo_position_command", 10, std::bind(&MPCNode::VescServoCallback, this, std::placeholders::_1));
        // reference_sub_ = nh.subscribe("reference_trajectory", 10, &MPCNode::referenceCallback, this);
        create_reference_trajectory();

        if (mpc_model_acados_create(capsule) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create ACADOS solver.");
            rclcpp::shutdown();
        }

        // Initialize ACADOS solver
        nlp_config_ = mpc_model_acados_get_nlp_config(capsule);
        nlp_dims_ = mpc_model_acados_get_nlp_dims(capsule);
        nlp_in_ = mpc_model_acados_get_nlp_in(capsule);
        nlp_out_ = mpc_model_acados_get_nlp_out(capsule);
        nlp_solver_ = mpc_model_acados_get_nlp_solver(capsule);
        nlp_opts_ = mpc_model_acados_get_nlp_opts(capsule);

        // Set weights for the cost function
        //std::vector<double> cost_weights = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
        /* std::vector<double> cost_weights = {10.0, 10.0, 1.0, 1.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);
        W.diagonal() << cost_weights[0], cost_weights[1], cost_weights[2], cost_weights[3], cost_weights[4], cost_weights[5];
        for (size_t k = 0; k < N; k++)
        {
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W", W.data());
        } */
        // Set final cost weights
        //std::vector<double> final_cost_weights = {1.0, 1.0, 0.0, 0.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
        /* std::vector<double> final_cost_weights = {10.0, 10.0, 1.0, 1.0}; // Example weights for x, y, sin(psi), cos(psi), steering, velocity
        Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(4, 4);
        Wf.diagonal() << final_cost_weights[0], final_cost_weights[1], final_cost_weights[2], final_cost_weights[3];
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "W", Wf.data()); */

        //nlp_config_ = ocp_nlp_config_create(nlp_dims_);
        //nlp_dims_ = ocp_nlp_dims_create(nlp_config_);
        //nlp_in_ = ocp_nlp_in_create(nlp_config_, nlp_dims_);
        //nlp_out_ = ocp_nlp_out_create(nlp_config_, nlp_dims_);
        //nlp_opts_ = ocp_nlp_solver_opts_create(nlp_config_, nlp_dims_);
        //nlp_solver_ = ocp_nlp_solver_create(nlp_config_, nlp_dims_, nlp_opts_, nlp_in_);

        //precompute
        //ocp_nlp_precompute(nlp_solver_, nlp_in_, nlp_out_);

        // Set up a timer to solve MPC at a fixed frequency
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)),
            std::bind(&MPCNode::MPC_timerCallback, this));
    }

    ~MPCNode()
    {
        mpc_model_acados_free(capsule);
    }

    // Update the callback function signatures
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract state from odometry message
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_state_.yaw = tf2::getYaw(q); // Correctly extract yaw from quaternion
        current_state_.v = msg->twist.twist.linear.x;
        //RCLCPP_INFO(this->get_logger(), "Current state: x=%f, y=%f, yaw=%f, v=%f", current_state_.x, current_state_.y, current_state_.yaw, current_state_.v);
    }

    void VescServoCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Extract servo position from VESC message
        current_state_.theta = (msg->data - steering_angle_to_servo_offset) / steering_angle_to_servo_gain;
        //RCLCPP_INFO(this->get_logger(), "Current servo position: %f", current_state_.theta);
    }

    /* void referenceCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        reference_trajectory_ = msg->data;
    } */

    void create_reference_trajectory()
    {
// Create a simple reference trajectory for testing
#define N_total 10000

        double n = (2 * M_PI) / N_total;
        double x_traj[N_total], y_traj[N_total], psi_traj[N_total];
        size_t index = 0;

        // Generate x and y trajectory
        for (double t = 0; t <= 2 * M_PI && index < N_total; t += n, ++index)
        {
            x_traj[index] = 2.5 * cos(t);
            y_traj[index] = -1.75 - 1.75 * sin(t);
        }

        // Calculate heading (psi) between consecutive points
        for (size_t i = 0; i < N_total - 1; ++i)
        {
            double dx = x_traj[i + 1] - x_traj[i];
            double dy = y_traj[i + 1] - y_traj[i];
            psi_traj[i] = atan2(dy, dx); // This is in radians
        }

        // Repeat the last heading value to match the size of x_traj and y_traj
        if (N_total > 1)
        {
            psi_traj[N_total - 1] = psi_traj[N_total - 2];
        }

        reference_trajectory_.x = std::vector<double>(x_traj, x_traj + N_total);
        reference_trajectory_.y = std::vector<double>(y_traj, y_traj + N_total);
        reference_trajectory_.yaw = std::vector<double>(psi_traj, psi_traj + N_total);

        // Publish the reference trajectory for visualization
        nav_msgs::msg::Path ref_path;
        ref_path.header.stamp = this->get_clock()->now();
        ref_path.header.frame_id = "odom_vesc";
        for (size_t i = 0; i < N_total; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = ref_path.header;
            pose.pose.position.x = reference_trajectory_.x[i];
            pose.pose.position.y = reference_trajectory_.y[i];
            tf2::Quaternion q;
            q.setRPY(0, 0, reference_trajectory_.yaw[i]);
            pose.pose.orientation = tf2::toMsg(q);
            ref_path.poses.push_back(pose);
        }
        reference_trajectory_pub_->publish(ref_path);
    }

    void MPC_timerCallback()
    {
        solveMPC();
    }

    void solveMPC()
    {
        // int N = mpc_model_acados_get_N(capsule);

        if (std::isnan(current_state_.x) || std::isnan(current_state_.y) || reference_trajectory_.x.empty() || reference_trajectory_.y.empty())
        {
            RCLCPP_WARN(this->get_logger(), "State or reference trajectory is empty. Skipping MPC solve.");
            return;
        }

        // 1. Find the closest point to the current position
        std::vector<double> distances(reference_trajectory_.x.size());
        for (size_t i = 0; i < reference_trajectory_.x.size(); ++i)
        {
            distances[i] = std::sqrt(std::pow(reference_trajectory_.x[i] - current_state_.x, 2) +
                                     std::pow(reference_trajectory_.y[i] - current_state_.y, 2));
        }
        auto idx_ref_start = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));

        // 2. Compute reference points by walking along the trajectory
        std::vector<double> x_ref_step(N), y_ref_step(N), psi_ref(N);
        size_t idx = idx_ref_start;
        double dist_accum = 0.0;

        for (size_t i = 0; i < N; ++i)
        {
            double target_dist = i * 1.0 / frequency * v_ref;
            while (dist_accum < target_dist)
            {
                size_t next_idx = (idx + 1) % reference_trajectory_.x.size();
                double dx = reference_trajectory_.x[next_idx] - reference_trajectory_.x[idx];
                double dy = reference_trajectory_.y[next_idx] - reference_trajectory_.y[idx];
                dist_accum += std::sqrt(dx * dx + dy * dy);
                idx = next_idx;
            }

            x_ref_step[i] = reference_trajectory_.x[idx];
            y_ref_step[i] = reference_trajectory_.y[idx];
            psi_ref[i] = reference_trajectory_.yaw[idx];
        }

        // Publish the reference path for visualization
        nav_msgs::msg::Path ref_path;
        ref_path.header.stamp = this->get_clock()->now();
        ref_path.header.frame_id = "odom_vesc";
        for (size_t i = 0; i < N; ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = ref_path.header;
            pose.pose.position.x = x_ref_step[i];
            pose.pose.position.y = y_ref_step[i];
            tf2::Quaternion q;
            q.setRPY(0, 0, psi_ref[i]);
            pose.pose.orientation = tf2::toMsg(q);
            ref_path.poses.push_back(pose);
        }
        ref_path_pub_->publish(ref_path);

        // 5. Set up the ACADOS solver
        for (size_t k = 0; k < N; k++)
        {
            std::vector<double> yref_k = {
                x_ref_step[k],
                y_ref_step[k],
                std::sin(psi_ref[k]),
                std::cos(psi_ref[k]),
                0.0,
                v_ref};
            ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "y_ref", yref_k.data());
        }

         // set yref for the last stage
        std::vector<double> yref_N = {
            x_ref_step[9],
            y_ref_step[9],
            std::sin(psi_ref[9]),
            std::cos(psi_ref[9])};
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "y_ref", yref_N.data());   

        // Set initial state constraints
        std::vector<double> current_state_vector_ = {
            current_state_.x,
            current_state_.y,
            current_state_.yaw,
            current_state_.theta,
            current_state_.v};
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", current_state_vector_.data());
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", current_state_vector_.data());
        RCLCPP_INFO(this->get_logger(), "Current state vector: %f, %f, %f, %f, %f", current_state_vector_[0], current_state_vector_[1], current_state_vector_[2], current_state_vector_[3], current_state_vector_[4]);
        RCLCPP_INFO(this->get_logger(), "Reference state vector: %f, %f, %f, %f, %f", current_state_.x, current_state_.y, current_state_.yaw, current_state_.theta, current_state_.v);

        // Solve the MPC problem
        //int status = ocp_nlp_solve(nlp_solver_, nlp_in_, nlp_out_);
        int status = mpc_model_acados_solve(capsule);
        //mpc_model_acados_print_stats(capsule);
        /* std::vector<double> xtraj((N + 1) * 6);
        std::vector<double> utraj(N * 2);
        for (int ii = 0; ii <= N; ii++)
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii*6]);
        for (int ii = 0; ii < N; ii++)
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii*2]);

        printf("\n--- xtraj ---\n");
        d_print_exp_tran_mat( 6, N+1, xtraj.data(), 6);
        printf("\n--- utraj ---\n");
        d_print_exp_tran_mat( 2, N, utraj.data(), 2 ); */
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ACADOS solver failed with status %d", status);
            return;
        }

        // Get control output
        std::array<double, 2> control_output;
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", control_output.data()); // Retrieve control output

        double solved_time;
        ocp_nlp_get(nlp_solver_, "time_tot", &solved_time);

        std_msgs::msg::Float64 solved_time_msg;
        solved_time_msg.data = solved_time;
        solved_time_pub_->publish(solved_time_msg);

        ackermann_msgs::msg::AckermannDriveStamped control_vesc_msg;
        control_vesc_msg.header.stamp = this->get_clock()->now();
        control_vesc_msg.header.frame_id = "base_link";
        control_vesc_msg.drive.steering_angle = control_output[0] * steering_angle_to_servo_gain + steering_angle_to_servo_offset;
        control_vesc_msg.drive.acceleration = control_output[1] * speed_to_duty;

        // Publish control output
        control_vesc_pub_->publish(control_vesc_msg);

        // Publish state array
        std_msgs::msg::Float64MultiArray state_array_msg;
        state_array_msg.data = {current_state_vector_[0],
                                current_state_vector_[1],
                                current_state_vector_[2],
                                current_state_vector_[3],
                                current_state_vector_[4]};
        state_pub_->publish(state_array_msg);

        // Publish control array
        std_msgs::msg::Float64MultiArray control_array_msg;
        control_array_msg.data = {control_output[0], control_output[1]};
        control_pub_->publish(control_array_msg);

        // Publish control vector
        std::vector<double> control_vector(N * 2);
        for (int ii = 0; ii < N; ii++)
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &control_vector[ii * 2]);
        std_msgs::msg::Float64MultiArray control_vector_msg;
        control_vector_msg.data = control_vector;
        control_vector_pub_->publish(control_vector_msg);

        // Publish state vector
        std::vector<double> state_vector((N + 1) * 6);
        for (int ii = 0; ii <= N; ii++)
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &state_vector[ii * 6]);
        std_msgs::msg::Float64MultiArray state_vector_msg;
        state_vector_msg.data = state_vector;
        state_vector_pub_->publish(state_vector_msg);
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_vesc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solved_time_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_vector_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_vector_pub_;
    

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vesc_servo_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    mpc_model_solver_capsule *capsule = mpc_model_acados_create_capsule();

    double speed_to_erpm_gain = 4277.5;
    double speed_to_duty = 0.0602;                 //(m/s) / (duty cycle)
                                                   // servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
    double steering_angle_to_servo_gain = -0.840;  // -0.6984, -1.2135
    double steering_angle_to_servo_offset = 0.475; // right turn is positive

    double frequency = 10.0; // Frequency in Hz
    double v_ref = 1.0;      // Reference speed in m/s

    struct State
    {
        double x;
        double y;
        double yaw;
        double theta;
        double v;
    } current_state_;


    struct ReferenceTrajectory
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;
    } reference_trajectory_;

    std::array<std::reference_wrapper<std::vector<double>>, 3> reference_trajectory_vector_ = {
        reference_trajectory_.x,
        reference_trajectory_.y,
        reference_trajectory_.yaw};

    // ACADOS variables
    ocp_nlp_config *nlp_config_;
    ocp_nlp_dims *nlp_dims_;
    ocp_nlp_in *nlp_in_;
    ocp_nlp_out *nlp_out_;
    ocp_nlp_solver *nlp_solver_;
    void *nlp_opts_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}