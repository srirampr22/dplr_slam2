#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
// #include <>
// apriltag_ros/AprilTagDetectionArray
#include <std_srvs/Trigger.h> 
#define _USE_MATH_DEFINES
#include <cmath>


class LoopClosureDetection
{
public:
    tf::TransformListener listener;
    Eigen::VectorXd last_pose_vec_ = Eigen::VectorXd::Zero(6);
    tf::StampedTransform tf_tag_clink_0_; // to store initial pose of tag_0 wrt camera_link

    // Correct service client declaration
    ros::ServiceClient client;

    // Correct service request and response objects
    std_srvs::Trigger srv;
    

    bool traj_first_flag_;
    bool tf_first_flag_;
    int tf_skip_counter_;
    bool is_static_ = false;
    int traj_skip_counter_;
    int tag_skip_counter_;
    
    bool checkThreshold(const tf::Transform& transform) {
        double position_thresh = 0.1;
        double angl_threshhold = 0.05;
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();

        tf::Matrix3x3 rotationMatrix(transform.getRotation());
        double roll, pitch, yaw;
        rotationMatrix.getRPY(roll, pitch, yaw);
        yaw += M_PI;
        // ROS_INFO("x, y, z, R, P, Y: ", x,y,z,roll, pitch, yaw);
        // std::cout<< x<< " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
        // std::cout<<roll << " " << pitch << " " << yaw << std::endl;

        bool belowThreshold = (fabs(x) < position_thresh) &&
                            (fabs(y) < position_thresh) &&
                            (fabs(z) < position_thresh) &&
                            (fabs(roll) < angl_threshhold) &&
                            (fabs(pitch) < angl_threshhold) &&
                            (fabs(yaw) < angl_threshhold);

        return belowThreshold;
    }
    
    bool checkThreshold(std::vector<double> diffvec) {
        double position_thresh = 0.15;
        double angl_threshhold = 0.15; //increase angel_threshold more
        double x = diffvec[0];
        double y = diffvec[1];
        double z = diffvec[2];

        double roll = diffvec[3];
        double pitch = diffvec[4];
        double yaw = diffvec[5];

        // yaw += M_PI;

        bool belowThreshold = (fabs(x) < position_thresh) &&
                            (fabs(y) < position_thresh) &&
                            (fabs(z) < position_thresh) &&
                            (fabs(roll) < angl_threshhold) &&
                            (fabs(pitch) < angl_threshhold) &&
                            (fabs(yaw) < angl_threshhold);

        // std::cout<<belowThreshold<<std::endl;

        return belowThreshold;
    }

    LoopClosureDetection(ros::NodeHandle &nh) : nh_(nh)
    {
        // tf::TransformListener listener;
        counter1_sub_ = nh_.subscribe("/ssl_slam2/trajectory", 10, &LoopClosureDetection::trajectoryCallback, this);
        counter2_sub_ = nh_.subscribe("/tag_detections", 10, &LoopClosureDetection::tagCallback, this);

        // save_map = nh_.serviceClient<std_srvs::Empty>("save_map");
        client = nh_.serviceClient<std_srvs::Trigger>("save_map");

        traj_first_flag_ = true;
        tf_skip_counter_ = 0;
        traj_skip_counter_ = 0;
        tag_skip_counter_ = 0;
        tf_first_flag_ = true;
    }

    std::vector<double> getAbsTFdiff(tf::StampedTransform& transform1, tf::StampedTransform& transform2) {
        double x1 = transform1.getOrigin().x();
        double y1 = transform1.getOrigin().y();
        double z1= transform1.getOrigin().z();

        tf::Matrix3x3 rotationMatrix1(transform1.getRotation());
        double roll1, pitch1, yaw1;
        rotationMatrix1.getRPY(roll1, pitch1, yaw1);
        // yaw1 += M_PI;

        double x2 = transform2.getOrigin().x();
        double y2 = transform2.getOrigin().y();
        double z2= transform2.getOrigin().z();

        tf::Matrix3x3 rotationMatrix2(transform2.getRotation());
        double roll2, pitch2, yaw2;
        rotationMatrix2.getRPY(roll2, pitch2, yaw2);
        // yaw1 += M_PI;

        std::vector<double> delta_vec;
        delta_vec.push_back(fabs(x1-x2));
        delta_vec.push_back(fabs(y1-y2));
        delta_vec.push_back(fabs(z1-z2));
        delta_vec.push_back(fabs(roll1-roll2));
        delta_vec.push_back(fabs(pitch1-pitch2));
        delta_vec.push_back(fabs(yaw1-yaw2));

        return delta_vec;
    }

    void printTF(tf::StampedTransform& transform, std::string tfname) {
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();

        tf::Matrix3x3 rotationMatrix(transform.getRotation());
        double roll, pitch, yaw;
        rotationMatrix.getRPY(roll, pitch, yaw);
        yaw += M_PI;
        // ROS_INFO("x, y, z, R, P, Y: ", x,y,z,roll, pitch, yaw);
        std::cout << tfname << ": " 
                << "x=" << x << " "
                << "y=" << y << " "
                << "z=" << z << " "
                << "roll=" << roll << " "
                << "pitch=" << pitch << " "
                << "yaw=" << yaw << std::endl;
    }

    void trajectoryCallback(const nav_msgs::Path::ConstPtr &traj_msg)
    {
        // ROS_INFO("In callback trajectoryCallback.");
        // sample every 10msgs because traj is at 10Hz
        if ((traj_skip_counter_ < 5))
        {   
            traj_skip_counter_ ++ ; // increment by +1
            return;
        }
        

        Eigen::VectorXd pose_latest = Eigen::VectorXd::Zero(6);
        
        if (!traj_msg->poses.empty())
        {
            // Get the latest pose
            // ROS_INFO("checkpoint 1");
            auto latest_pose_stamped = traj_msg->poses.back();
            // ROS_INFO("checkpoint 2");
            // Extract position (x, y, z)
            double x = latest_pose_stamped.pose.position.x;
            double y = latest_pose_stamped.pose.position.y;
            double z = latest_pose_stamped.pose.position.z;

            // Extract orientation (quaternion)
            tf::Quaternion quat;
            tf::quaternionMsgToTF(latest_pose_stamped.pose.orientation, quat);
            // ROS_INFO("checkpoint 3");
            // Convert quaternion to RPY (roll, pitch, yaw)
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // ROS_INFO("checkpoint 4");
            // Clear the previous pose and update with the latest pose
            pose_latest << x, y, z, roll, pitch, yaw;

            // ROS_INFO("checkpoint 5");
            if (traj_first_flag_)
            {
                last_pose_vec_ = pose_latest;
                traj_first_flag_ = false;
            }

        }
        // else {
        //     ROS_WARN("No poses available in trajectory.");
        // }

        Eigen::VectorXd delta_vec = pose_latest - last_pose_vec_;


        for (int i = 0; i < delta_vec.size(); ++i) {
            if (std::abs(delta_vec(i)) <= 0.005)
            {
                // std::cout << "Static" << std::endl;
                is_static_ = true;
                break;
            }
            else {
                // std::cout << "Currently in motion, waiting for camera trajectory to catch up" << std::endl;
                is_static_ = false;
            }
        }
        
        if (!(traj_first_flag_))
        {
           last_pose_vec_ = pose_latest;
        }
        
        traj_skip_counter_ = 0; // reset to 0
        
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        if ((tag_skip_counter_ < 5))
        {   
            tag_skip_counter_ ++ ; // increment by +1
            return;
        }
        
        // ROS_INFO("In callback tagCallback.");
        tf::StampedTransform transform;
        // listener.lookupTransform("/tag_0", "/map", ros::Time(0), transform);
        try {
            listener.waitForTransform("/tag_0", "/camera_link", ros::Time(0), ros::Duration(1.0) );
            listener.lookupTransform("/tag_0", "/camera_link", ros::Time(0), transform);
            
            // skipping first 5 tf msgs to store initial pose of tag_0 wrt camera_link
            // if (tf_skip_counter_ < 10 && (tf_first_flag_))
            // {
            //     if (tf_skip_counter_ == 9)
            //     {
            //         tf_tag_clink_0_ = transform;
            //         std::cout << "checkpoint1" << std::endl;
            //         tf_first_flag_ = false;
            //         // tf_skip_counter_=0;
            //     }
            // }
            // tf_skip_counter_++;

            // Increment the counter for each message received
            tf_skip_counter_++;

            // Check if this is the initial assignment
            if (tf_first_flag_ && tf_skip_counter_ >= 10) {
                tf_tag_clink_0_ = transform;
                // std::cout << "Initial transform stored." << std::endl;
                tf_first_flag_ = false; // Ensure this block runs only once
            }

            if (is_static_ == true)
            {

                std::vector<double> diffvec = getAbsTFdiff(transform, tf_tag_clink_0_);

                for (double value : diffvec) {
                    std::cout << value << " ";
                }

                if (checkThreshold(diffvec))
                {
                    // Send rosservice call save map
                    ROS_INFO("STATIC AND LC TRUE");

                    // Call the service
                    if (client.call(srv)) {
                        ROS_INFO("Successfully called save_map service: %s", srv.response.message.c_str());
                    } else {
                        ROS_ERROR("Failed to call save_map service");
                    }
                }
                else{
                    ROS_INFO("STATIC AND LC FALSE");
                }
            }
            else{
                ROS_INFO("NOT STATIC");
            }


        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        
        tag_skip_counter_ = 0; // reset to 0

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber counter1_sub_;
    ros::Subscriber counter2_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_closure_detector_activator");
    ros::NodeHandle nh;
    LoopClosureDetection talker_subscribers(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}