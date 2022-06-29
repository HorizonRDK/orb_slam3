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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include"include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM)
    {
        node_=rclcpp::Node::make_shared("ros2_mono");

        path_publisher=node_->create_publisher<nav_msgs::msg::Path>("camera_path",10);
        pointcloud2_publisher=node_->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud2",10);
        frame_publisher=node_->create_publisher<sensor_msgs::msg::Image>("frame",10);
    }

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    ORB_SLAM3::System* mpSLAM;

    std::shared_ptr<rclcpp::Node> node_;
    nav_msgs::msg::Path path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_publisher;

private:
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points);
    tf2::Transform TransformFromMat (cv::Mat position_mat);
};


sensor_msgs::msg::PointCloud2 ImageGrabber::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

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

            data_array[0] = (float)map_points.at(i)->GetWorldPos()(2); //x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = (float)(-1.0* map_points.at(i)->GetWorldPos()(0)); //y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = (float)(-1.0* map_points.at(i)->GetWorldPos()(1)); //z. Do the transformation by just reading at the position of y instead of z
            //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }    


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);

    ImageGrabber igb(&SLAM);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img0 =
            igb.node_->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw",1,std::bind(&ImageGrabber::GrabImage,&igb,std::placeholders::_1));

    rclcpp::spin(igb.node_);
    std::cout << "call back fail " << std::endl;
    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img);
    }
    catch (cv_bridge::Exception& e)
    {
        //ROS_ERROR("cv_bridge exception: %s", e.what());
        std::cout << "GrabImage fail " << std::endl;
        return;
    }
    

    Sophus::SE3f Tcw_SE3F = mpSLAM->TrackMonocular(cv_ptr->image,rclcpp::Time(cv_ptr->header.stamp).seconds());


    //1.显示位姿，以TF方式发布出去 map->camera_link
    cv::Mat Tcw;
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3F.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);

    tf2::Transform tf_transform = TransformFromMat(Tcw);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "camera_link";
    tf_msg.header.stamp=node_->get_clock()->now();
    tf_msg.transform = tf2::toMsg(tf_transform);

    tf2_ros::TransformBroadcaster tf_broadcaster(node_);
    tf_broadcaster.sendTransform(tf_msg);


    //2.显示path
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x=tf_transform.getOrigin().getX();
    pose_msg.pose.position.y=tf_transform.getOrigin().getY();
    pose_msg.pose.position.z=tf_transform.getOrigin().getZ();

    pose_msg.pose.orientation.x=tf_transform.getRotation().getX();
    pose_msg.pose.orientation.y=tf_transform.getRotation().getY();
    pose_msg.pose.orientation.z=tf_transform.getRotation().getZ();
    pose_msg.pose.orientation.w=tf_transform.getRotation().getW();

    path.header=tf_msg.header;
    path.poses.push_back(pose_msg);

    path_publisher->publish(path);

    //3.显示map，pointcloud2发布
    sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud (mpSLAM->GetAllMapPoints());
    pointcloud2_publisher->publish(cloud);

    //4.显示带有特征点的图像
    cv::Mat toShow = mpSLAM->GetmpFrameDrawe()->DrawFrame(1.0f);
    sensor_msgs::msg::Image img_msg;
    cv_bridge::CvImage img_bridge;
    img_bridge=cv_bridge::CvImage(tf_msg.header,"bgr8",toShow);
    img_bridge.toImageMsg(img_msg);
    frame_publisher->publish(img_msg);


}

