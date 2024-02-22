#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class Answer : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr pointPublisher;

    static bool calculateSlopeAndIntercept(cv::Point points[2], int row, int col, float &slope, float &intercept) {
        static bool isSecondPoint = false;
        points[isSecondPoint].x = col;
        points[isSecondPoint].y = row;
        if (isSecondPoint) {
            if (points[1].x - points[0].x == 0)
                return true;
            slope = (points[1].y - points[0].y) / (float) (points[1].x - points[0].x);
            intercept = points[0].y - slope * points[0].x;
            isSecondPoint = false;
            return true;
        }
        isSecondPoint = true;
        return false;
    }

    static void detectLine(cv::Mat &image, float &slope, float &intercept) {
        int rowEnd, colEnd, row, col;
        cv::Point points[2];
        rowEnd = image.rows - 1;
        colEnd = image.cols - 1;
        for (row = 0; row <= rowEnd; row++)
            if (image.at<cv::Vec3b>(row, 0)[2] > 250) {
                calculateSlopeAndIntercept(points, row, 0, slope, intercept);
                break;
            }
        for (row = rowEnd; row >= 0; row--)
            if (image.at<cv::Vec3b>(row, colEnd)[2] > 250) {
                if (calculateSlopeAndIntercept(points, row, colEnd, slope, intercept))
                    return;
                break;
            }
        for (col = 0; col <= colEnd; col++)
            if (image.at<cv::Vec3b>(rowEnd, col)[2] > 250) {
                if (col <= colEnd && calculateSlopeAndIntercept(points, rowEnd, col, slope, intercept))
                    return;
                break;
            }
        for (col = colEnd; col >= 0; col--)
            if (image.at<cv::Vec3b>(0, col)[2] > 250)
                calculateSlopeAndIntercept(points, 0, col, slope, intercept);
    }

    static bool detectBlock(cv::Mat &image, geometry_msgs::msg::Point32 &clickPoint, float &slope, float &intercept) {
        int x, colEnd, rowEnd, counter = 0;
        float y;
        colEnd = image.cols - 1;
        rowEnd = image.rows - 1;
        for (x = 0, y = intercept - 16; x <= colEnd; x++, y += slope) {
            if (y < 0 || y > rowEnd)
                continue;
            if (image.at<cv::Vec3b>(y, x)[2] < 50) {
                if (++counter < 32)
                    continue;
                clickPoint.x = x;
                clickPoint.y = y + 16;
                return true;
            }
        }
        return false;
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr rosImage) {
        float slope, intercept;
        geometry_msgs::msg::Point32 clickPoint;
        slope = intercept = 0;
        cv_bridge::CvImageConstPtr pCvImage = cv_bridge::toCvShare(rosImage, sensor_msgs::image_encodings::BGR8);
        cv::Mat cvImage = pCvImage->image;
        detectLine(cvImage, slope, intercept);
        if (slope <= 0 && intercept <= 0)
            return;
        if (detectBlock(cvImage, clickPoint, slope, intercept))
            this->pointPublisher->publish(clickPoint);
    }

public:
    explicit Answer(const std::string &nodeName) : Node(nodeName) {
        imageSubscriber = create_subscription<sensor_msgs::msg::Image>("/raw_image", 10,
                                                                       std::bind(&Answer::callback, this,
                                                                                 std::placeholders::_1));
        pointPublisher = create_publisher<geometry_msgs::msg::Point32>("/click_position", 10);
    }
};
