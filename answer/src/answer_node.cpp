#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class Answer : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;

    static bool calculateSlopeAndIntercept(cv::Point points[2], int row, int col, float& slope, float& intercept){
        static bool isSecondPoint = false;
        points[isSecondPoint].x = col;
        points[isSecondPoint].y = row;
        if(isSecondPoint) {
            slope = (points[1].y - points[0].y) / (float) (points[1].x - points[0].x);
            intercept = points[0].y - slope * points[0].x;
            isSecondPoint = false;
            return true;
        }
        isSecondPoint = true;
        return false;
    }

    static void detectLine(cv::Mat& image, float& slope, float& intercept) {
        static int rowEnd, colEnd, row, col;
        static cv::Point points[2];
        rowEnd = image.rows - 1;
        colEnd = image.cols - 1;
        for(row = 0; row <= rowEnd; row++)
            if(image.at<cv::Vec3b>(row, 0)[0] == 255){
                calculateSlopeAndIntercept(points, row, 0, slope, intercept);
                break;
            }
        for(row = rowEnd; row >= 0; row--)
            if(image.at<cv::Vec3b>(row, colEnd)[0] == 255){
                if(calculateSlopeAndIntercept(points, row, colEnd, slope, intercept))
                    return;
                break;
            }
        for(col = 0; col <= colEnd; col++)
            if(image.at<cv::Vec3b>(rowEnd, col)[0] == 255){
                if (col <= colEnd && calculateSlopeAndIntercept(points, rowEnd, col, slope, intercept))
                    return;
                break;
            }
        for(col = colEnd; col >= 0; col--)
            if(image.at<cv::Vec3b>(0, col)[0] == 255)
                calculateSlopeAndIntercept(points, 0, col, slope, intercept);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr rosImage) {
        float slope, intercept;
        slope = intercept = 0;
        cv_bridge::CvImageConstPtr pCvImage = cv_bridge::toCvShare(rosImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat cvImage = pCvImage->image;
        detectLine(cvImage, slope, intercept);
        if(slope > 0 || intercept > 0){
            std::cout << slope << " " << intercept << std::endl;
        }
    }

public:
    explicit Answer(const std::string& nodeName) : Node(nodeName) {
        imageSubscriber = create_subscription<sensor_msgs::msg::Image>("/raw_image", 10,
                                                                       std::bind(&Answer::callback, this, std::placeholders::_1));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto answerNode = std::make_shared<Answer>("answer_node");
    rclcpp::spin(answerNode);
    rclcpp::shutdown();
    return 0;
}
