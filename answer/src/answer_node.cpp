#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class Answer : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;

    static void callback(const sensor_msgs::msg::Image::SharedPtr rosImage) {
        cv_bridge::CvImageConstPtr pCvImage = cv_bridge::toCvShare(rosImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat cvImage = pCvImage->image;
        cv::imshow("Subscription Image", cvImage);
        cv::waitKey(1);
    }

public:
    Answer(const std::string nodeName) : Node(nodeName) {
        imageSubscriber = create_subscription<sensor_msgs::msg::Image>("/raw_image", 10,
                                                                       callback);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto answerNode = std::make_shared<Answer>("answer_node");
    RCLCPP_INFO(answerNode->get_logger(), "大家好，我是King!");
    rclcpp::spin(answerNode);
    rclcpp::shutdown();
    return 0;
}
