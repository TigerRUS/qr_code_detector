#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <unordered_set>
#include <unistd.h>
#include <iomanip>
#include <cmath>

class QRPositionLogger {
public:
    QRPositionLogger() : tf_listener_() {
        std::string output_file_path;
        nh_.param<std::string>("output_file", output_file_path, "/root/ros_files/qr_coordinates.txt");
        nh_.param<std::string>("qr_topic", qr_topic_, "/qr_detector/qr_id");

        output_file_.open(output_file_path, std::ios::trunc);
        
        if (!output_file_.is_open()) {
            ROS_ERROR("Failed to create output file at: %s", output_file_path.c_str());
        }

        buzzer_pub_ = nh_.advertise<std_msgs::Bool>("Buzzer", 1);
        
        qr_sub_ = nh_.subscribe(qr_topic_, 10, &QRPositionLogger::qrCallback, this);
        ROS_INFO("Listening for QR codes on topic: %s", qr_topic_.c_str());
    }

    void qrCallback(const std_msgs::String::ConstPtr& msg) {
        const std::string& qr_id = msg->data;
        
        if (processed_qrs_.count(qr_id)) {
            return;
        }

        try {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double yaw = tf::getYaw(transform.getRotation());

            x = std::round(x * 10) / 10;
            y = std::round(y * 10) / 10;
            yaw = std::round(yaw * 10) / 10;

            if (output_file_.is_open()) {
                output_file_ << qr_id << ":" << std::fixed << std::setprecision(1) << x << " " << y << " " << yaw << std::endl;
                processed_qrs_.insert(qr_id);
                ROS_INFO("Saved position for QR: %s (%.1f, %.1f, %.1f)", qr_id.c_str(), x, y, yaw);
                
                std_msgs::Bool buzzer_msg;
                for (int i = 0; i < 3; ++i) {
                    buzzer_msg.data = true;
                    buzzer_pub_.publish(buzzer_msg);
                    usleep(200000);
                    
                    buzzer_msg.data = false;
                    buzzer_pub_.publish(buzzer_msg);
                    usleep(200000);
                }
            }
        } catch (const tf::TransformException& ex) {
            ROS_WARN("TF error: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber qr_sub_;
    ros::Publisher buzzer_pub_;
    tf::TransformListener tf_listener_;
    std::ofstream output_file_;
    std::unordered_set<std::string> processed_qrs_;
    std::string qr_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "qr_position_logger");
    QRPositionLogger logger;
    ros::spin();
    return 0;
}