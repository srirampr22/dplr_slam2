
//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <omp.h>

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

class Image2PCnode
{
private:
    ros::Subscriber subLaserCloud;
    ros::Subscriber yellowMaskSub;

    // message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    // message_filters::Subscriber<sensor_msgs::Image> yellowMaskSub;
    // message_filters::Synchronizer<MySyncPolicy> sync;

    ros::Subscriber cameraInfoSub;
    ros::Publisher pclpublisher;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed {new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_mapped_pcl {new pcl::PointCloud<pcl::PointXYZRGB>};
    cv::Mat yellow_mask;

    Eigen::Matrix3f rotation_matrix;
    Eigen::Vector3f translation;

    double fx, fy, cx, cy;

public:
    Image2PCnode()

    {   
        ros::NodeHandle nh;
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, &Image2PCnode::velodyneHandler, this);
        yellowMaskSub = nh.subscribe<sensor_msgs::Image>("/mean_point_mask", 100, &Image2PCnode::yellowMaskHandler, this);
        cameraInfoSub = nh.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 100, &Image2PCnode::imageInfoCallback, this);
        pclpublisher = nh.advertise<sensor_msgs::PointCloud2>("/colormapped_pointcloud", 100);

        translation = Eigen::Vector3f(0.001, 0.014, -0.007);
        Eigen::Quaternionf q(1.00, -0.012, -0.001, -0.003);
        q.normalize();
        rotation_matrix = q.toRotationMatrix();
    }

    void imageInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfo) {
        fx = cameraInfo->K[0];
        fy = cameraInfo->K[4];
        cx = cameraInfo->K[2];
        cy = cameraInfo->K[5];
    }

    void yellowMaskHandler(const sensor_msgs::ImageConstPtr& msg)
    {
        // ROS_INFO("Received yellow mask message");

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            yellow_mask = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // ROS_INFO("Received PointCloud2 message");

        pcl::fromROSMsg(*msg, *cloud);
        
        // Transform point cloud
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(rotation_matrix);
        transform.translation() << translation;
        pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

        if (!yellow_mask.empty())
        {
            *color_mapped_pcl = *cloud_transformed;

            #pragma omp parallel for // This is the OpenMP directive to parallelize the loop
            for (size_t i = 0; i < color_mapped_pcl->points.size(); ++i)
            {
                auto &point = color_mapped_pcl->points[i];
                int pixel_x = int((point.x / point.z) * fx + cx);
                int pixel_y = int((point.y / point.z) * fy + cy);

                // point.r = 0;
                // point.g = 255;
                // point.b = 0;

                if (pixel_x >= 0 && pixel_x < yellow_mask.cols && pixel_y >= 0 && pixel_y < yellow_mask.rows)
                {
                    // if (yellow_mask.at<uchar>(pixel_y, pixel_x) > 0)
                    // ROS_INFO("1");
                    cv::Vec3b pixelColor = yellow_mask.at<cv::Vec3b>(pixel_y, pixel_x);  // Get the BGR pixel
                    if (pixelColor[2] == 255 && pixelColor[1] == 0 && pixelColor[0] == 0)  // Check if it's red
                    {
                        point.r = 0;
                        point.g = 255;
                        point.b = 0;
                        // i want to print a message in the terminal saying "SEEN"
                        ROS_INFO("2");
                        
                    }
                    
                }
            }

            // Reverse transformation to get it back to original coordinates
            Eigen::Affine3f inverse_transform = transform.inverse();
            pcl::transformPointCloud(*color_mapped_pcl, *color_mapped_pcl, inverse_transform);

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*color_mapped_pcl, output);
            output.header.frame_id = msg->header.frame_id;
            pclpublisher.publish(output);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ssl_slam2_image2pc");
    ROS_INFO("fvfg");
    Image2PCnode node;
    ros::spin();
    return 0;
}

