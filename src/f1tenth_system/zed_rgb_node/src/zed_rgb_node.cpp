#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp> 
#include <opencv2/core/cuda.hpp>
#include "videocapture.hpp" 


class ZedRGBPublisher : public rclcpp::Node
{
public:
    ZedRGBPublisher() : Node("zed_rgb_publisher")
    {
        // Parâmetros
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("fps", 30);

        int cam_id = this->get_parameter("camera_id").as_int();
        int fps = this->get_parameter("fps").as_int();

        sl_oc::video::FPS fps_enum;
        switch (fps)
        {
        case 15:
            fps_enum = sl_oc::video::FPS::FPS_15;
            break;
        case 30:
            fps_enum = sl_oc::video::FPS::FPS_30;
            break;
        case 60:
            fps_enum = sl_oc::video::FPS::FPS_60;
            break;
        case 100:
            fps_enum = sl_oc::video::FPS::FPS_100;
            break;
        default:
            std::cerr << "FPS inválido, usando 30 por defeito." << std::endl;
            fps_enum = sl_oc::video::FPS::FPS_30;
        }

        sl_oc::video::VideoParams params;
        params.res = sl_oc::video::RESOLUTION::HD720;
        params.fps = fps_enum;


        //sl_oc::video::VideoCapture cap_0(params);
        //camera_ = std::make_unique<sl_oc::video::VideoCapture>(cap_0);

        camera_ = std::make_unique<sl_oc::video::VideoCapture>(params);

        // Inicializar câmara
        if (!camera_->initializeVideo(cam_id))
        {
            RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a ZED 2i.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to camera sn: %d [%s]", camera_->getSerialNumber(), camera_->getDeviceName().c_str());


         // Create ROS publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw", 10);

        publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw_left", 10);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_raw_right", 10);


        // Timer to periodically grab frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            std::bind(&ZedRGBPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "CUDA disponível: %d dispositivos", cv::cuda::getCudaEnabledDeviceCount());

    }   

private:
    void timer_callback()
    {
        // Get last available frame
        const sl_oc::video::Frame frame = camera_->getLastFrame();
        if (frame.data == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "No frame available");
            return;
        }

        // 1. Criar Mat CPU a partir de frame.data
        cv::Mat frameYUV_cpu = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);

        // 3. Conversão de YUV422 (YUYV) para BGR na GPU
        cv::Mat frameBGR_cpu;
        cv::cvtColor(frameYUV_cpu, frameBGR_cpu, cv::COLOR_YUV2BGR_YUYV);

        // ----> Conversion from BGR to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frameBGR_cpu).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg); 

        return;

        cv::cuda::GpuMat frameBGR_gpu;
        frameBGR_gpu.upload(frameBGR_cpu); // Upload frame to GPU

        // 4. Separar os dois olhos diretamente na GPU
        int width = frameBGR_gpu.cols / 2;
        int height = frameBGR_gpu.rows;

        cv::cuda::GpuMat left_gpu(frameBGR_gpu, cv::Rect(0, 0, width, height));
        cv::cuda::GpuMat right_gpu(frameBGR_gpu, cv::Rect(width, 0, width, height));

        // 5. Fazer download para CPU apenas no final (necessário para publicar com cv_bridge)
        cv::Mat left, right;
        left_gpu.download(left);
        right_gpu.download(right);

        auto msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg();
        msg_left->header.stamp = this->now();
        publisher_left_->publish(*msg_left);

        auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right).toImageMsg();
        msg_right->header.stamp = this->now();
        publisher_right_->publish(*msg_right);

        // 6. Liberar memória GPU
        frameBGR_gpu.release();
        left_gpu.release();
        right_gpu.release();
    }

    std::unique_ptr<sl_oc::video::VideoCapture> camera_;  // Usar unique_ptr para gerenciar o objeto de câmera
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_, publisher_right_, publisher_left_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedRGBPublisher>());
    rclcpp::shutdown();
    return 0;
}
