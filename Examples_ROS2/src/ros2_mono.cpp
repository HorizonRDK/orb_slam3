/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <condition_variable>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "orb_slam3/System.h"


class ImageGrabber
{
public:
    explicit ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM_(pSLAM)
    {
        node_=rclcpp::Node::make_shared("ros2_mono");

        path_publisher_=node_->create_publisher<nav_msgs::msg::Path>("camera_path",10);
        pointcloud2_publisher_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud2",10);
        frame_publisher_=node_->create_publisher<sensor_msgs::msg::Image>("frame",10);
    }

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    void PubPose();
    void PubImage();
    void PubPointCloud();

    ORB_SLAM3::System* mpSLAM_;

    std::shared_ptr<rclcpp::Node> node_;
    nav_msgs::msg::Path path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_publisher_;

private:
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points);
    tf2::Transform TransformFromMat (cv::Mat position_mat);

    //显示相关成员变量
    queue<Sophus::SE3f> pose_buffer_;
    std::mutex pose_mutex_;
    std::condition_variable pose_cv_;

    queue<cv::Mat> image_buffer_;
    std::mutex image_mutex_;
    std::condition_variable image_cv_;

    queue<std::vector<ORB_SLAM3::MapPoint*>> point_buffer_;
    std::mutex point_mutex_;
    std::condition_variable point_cv_;

};


sensor_msgs::msg::PointCloud2 ImageGrabber::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points) {

    sensor_msgs::msg::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = node_->get_clock()->now();
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = map_points.size();  //点的个数
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float); //一个点占用存储空间
    cloud.row_step = cloud.point_step * cloud.width; //整个点云占用存储空间
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i=0; i<cloud.width; i++)
    {
        if (map_points.at(i)->nObs >= 2)
        {

            data_array[0] = (float)map_points.at(i)->GetWorldPos()(2);
            data_array[1] = (float)(-1.0* map_points.at(i)->GetWorldPos()(0));
            data_array[2] = (float)(-1.0* map_points.at(i)->GetWorldPos()(1));


            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

tf2::Transform ImageGrabber::TransformFromMat (cv::Mat position_mat) {
    cv::Mat rotation(3,3,CV_32F);
    cv::Mat translation(3,1,CV_32F);

    rotation = position_mat.rowRange(0,3).colRange(0,3);
    translation = position_mat.rowRange(0,3).col(3);


    tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                       rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                       rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
    );

    tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                        -1, 0, 0,
                                        0,-1, 0);

    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

    return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}


void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img);
    }
    catch (cv_bridge::Exception& e)
    {
        return;
    }

    Sophus::SE3f Tcw_SE3F = mpSLAM_->TrackMonocular(cv_ptr->image,rclcpp::Time(cv_ptr->header.stamp).seconds());

    std::unique_lock<std::mutex> locker_pose(pose_mutex_);
    pose_buffer_.push(Tcw_SE3F);
    locker_pose.unlock();
    pose_cv_.notify_one();

    std::unique_lock<std::mutex> locker_image(image_mutex_);
    image_buffer_.push(mpSLAM_->GetmpFrameDrawe()->DrawFrame(1.0f));
    locker_image.unlock();
    image_cv_.notify_one();

    std::unique_lock<std::mutex> locker_point(point_mutex_);
    point_buffer_.push(mpSLAM_->GetAllMapPoints());
    locker_point.unlock();
    point_cv_.notify_one();
}


void ImageGrabber::PubPose()
{
    Eigen::Matrix4f Tcw_Matrix;
    cv::Mat Tcw;
    geometry_msgs::msg::TransformStamped tf_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    tf2_ros::TransformBroadcaster tf_broadcaster(node_);

    while (rclcpp::ok())
    {
        std::unique_lock<std::mutex> locker_pose(pose_mutex_);
        while(pose_buffer_.empty())
            pose_cv_.wait(locker_pose);
        Tcw_Matrix = pose_buffer_.front().matrix();
        pose_buffer_.pop();
        locker_pose.unlock();

        cv::eigen2cv(Tcw_Matrix, Tcw);
        tf2::Transform tf_transform = TransformFromMat(Tcw);

        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "camera_link";
        tf_msg.header.stamp = node_->get_clock()->now();
        tf_msg.transform = tf2::toMsg(tf_transform);

        tf_broadcaster.sendTransform(tf_msg);

        pose_msg.pose.position.x = tf_transform.getOrigin().getX();
        pose_msg.pose.position.y = tf_transform.getOrigin().getY();
        pose_msg.pose.position.z = tf_transform.getOrigin().getZ();

        pose_msg.pose.orientation.x = tf_transform.getRotation().getX();
        pose_msg.pose.orientation.y = tf_transform.getRotation().getY();
        pose_msg.pose.orientation.z = tf_transform.getRotation().getZ();
        pose_msg.pose.orientation.w = tf_transform.getRotation().getW();

        path_.header = tf_msg.header;
        path_.poses.push_back(pose_msg);
        path_publisher_->publish(path_);

    }
}

void ImageGrabber::PubImage()
{
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage img_bridge;
    cv::Mat toshow;
    std_msgs::msg::Header header;

    while (rclcpp::ok())
    {
        std::unique_lock<std::mutex> locker_image(image_mutex_);
        while (image_buffer_.empty())
            pose_cv_.wait(locker_image);
        toshow = image_buffer_.front();
        image_buffer_.pop();
        locker_image.unlock();

        img_bridge = cv_bridge::CvImage(header, "bgr8", toshow);
        img_bridge.toImageMsg(img_msg);
        frame_publisher_->publish(img_msg);
    }
}


void ImageGrabber::PubPointCloud()
{
    std::vector<ORB_SLAM3::MapPoint *> orb_point;
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> locker_point(point_mutex_);
        while (point_buffer_.empty())
            point_cv_.wait(locker_point);
        orb_point = point_buffer_.front();
        point_buffer_.pop();
        locker_point.unlock();

        sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud(orb_point);
        pointcloud2_publisher_->publish(cloud);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc != 3)
    {
        cerr << endl << "Usage: ros2 run orb_slam3_example_ros2 mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);

    ImageGrabber igb(&SLAM);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0 =
            igb.node_->create_subscription<sensor_msgs::msg::Image>("/camera/infra1/image_rect_raw",1,std::bind(&ImageGrabber::GrabImage,&igb,std::placeholders::_1));

    std::thread pub_image_thread(&ImageGrabber::PubImage, &igb);
    std::thread pub_pose_thread(&ImageGrabber::PubPose, &igb);
    std::thread pub_pointcloud_thread(&ImageGrabber::PubPointCloud, &igb);

    rclcpp::spin(igb.node_);
    SLAM.Shutdown();
    rclcpp::shutdown();
    return 0;
}

