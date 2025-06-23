#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

class QRDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher image_pub_;
    ros::Publisher qr_id_pub_;
    tf::TransformListener tf_listener_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    zbar::ImageScanner scanner_;
	float qr_size;

public:
    QRDetector() : nh_("~") {
        std::string image_topic, camera_info_topic, output_topic;
        nh_.param<std::string>("image_topic", image_topic, "/camera/rgb/image_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/camera/rgb/camera_info");
        nh_.param<std::string>("output_topic", output_topic, "/qr_decode");
		nh_.param<float>("qr_size", qr_size, 0.04f);

        image_sub_ = nh_.subscribe(image_topic, 1, &QRDetector::imageCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &QRDetector::cameraInfoCallback, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic, 1);
        qr_id_pub_ = nh_.advertise<std_msgs::String>("qr_id", 10);

        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) {
            camera_matrix_.at<double>(i/3, i%3) = msg->K[i];
        }

        dist_coeffs_ = cv::Mat(1, msg->D.size(), CV_64F);
        for (size_t i = 0; i < msg->D.size(); ++i) {
            dist_coeffs_.at<double>(0, i) = msg->D[i];
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        processImage(cv_ptr->image);
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void processImage(cv::Mat& img) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        zbar::Image zbar_image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

        int n = scanner_.scan(zbar_image);

        if (n <= 0) return;

        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); 
             symbol != zbar_image.symbol_end(); ++symbol) {
            
            std::string qr_data = symbol->get_data();
            std::vector<cv::Point> points;
            
            for (int i = 0; i < symbol->get_location_size(); ++i) {
                points.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
            }

            drawQRContour(img, points, qr_data);
            
            if (!camera_matrix_.empty() && points.size() >= 4) {
                estimatePose(img, points, qr_data);
            }
        }
    }

    void drawQRContour(cv::Mat& img, const std::vector<cv::Point>& points, const std::string& qr_data) {
        std::vector<cv::Point> hull;
        cv::convexHull(points, hull);

        for (size_t j = 0; j < hull.size(); ++j) {
            cv::line(img, hull[j], hull[(j+1) % hull.size()], cv::Scalar(255, 255, 0), 3);
            cv::putText(img, std::to_string(j), hull[j], 0, 1, cv::Scalar(0, 255, 255), 2);
        }

        cv::Point center(0, 0);
        for (const auto& p : hull) {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= hull.size();
        center.y /= hull.size();

        cv::putText(img, qr_data, center, 0, 1, cv::Scalar(0, 0, 255), 2);
    }

    void estimatePose(cv::Mat& img, const std::vector<cv::Point>& points, const std::string& qr_data) {
        std::vector<cv::Point3f> model_points = {
            {0, 0, 0},      		// Top-left
            {qr_size, 0, 0},    	// Top-right
            {qr_size, qr_size, 0},  // Bottom-right
            {0, qr_size, 0}     	// Bottom-left
        };

        std::vector<cv::Point2f> image_points;
        for (const auto& p : points) {
            image_points.emplace_back(p.x, p.y);
        }

        cv::Mat rvec, tvec;
        cv::solvePnP(model_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

        // Project axes
        std::vector<cv::Point3f> axes = {{0.2, 0, 0}, {0, 0.2, 0}, {0, 0, -0.2}};
        std::vector<cv::Point2f> projected_axes;
        cv::projectPoints(axes, rvec, tvec, camera_matrix_, dist_coeffs_, projected_axes);

        // Draw axes
        cv::line(img, image_points[0], projected_axes[0], cv::Scalar(255, 0, 0), 3);
        cv::line(img, image_points[0], projected_axes[1], cv::Scalar(0, 255, 0), 3);
        cv::line(img, image_points[0], projected_axes[2], cv::Scalar(0, 0, 255), 3);

        // Calculate position in meters
        double qr_x = tvec.at<double>(0);
        double qr_y = tvec.at<double>(1);
        double qr_dist = tvec.at<double>(2);

        cv::putText(img, "x: " + std::to_string(qr_x), cv::Point(20, 40), 0, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(img, "y: " + std::to_string(qr_y), cv::Point(20, 80), 0, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(img, "dist: " + std::to_string(qr_dist), cv::Point(20, 120), 0, 1, cv::Scalar(0, 0, 255), 2);

		try {
			std_msgs::String id_msg;
			id_msg.data = qr_data;
			qr_id_pub_.publish(id_msg);
		} catch (tf::TransformException& ex) {
			ROS_WARN("TF exception: %s", ex.what());
		}
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "qr_detector_node");
    QRDetector qr_detector;
    ros::spin();
    return 0;
}