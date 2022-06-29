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
#include<time.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include"include/System.h"

using namespace std;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){

        node = rclcpp::Node::make_shared("ros2_stereo");
        path_publisher=node->create_publisher<nav_msgs::msg::Path>("camera_path",10);
        pointcloud2_publisher=node->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud2",10);
        frame_publisher=node->create_publisher<sensor_msgs::msg::Image>("frame",10);
    }

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,const sensor_msgs::msg::Image::ConstSharedPtr msgRight);


    ORB_SLAM3::System* mpSLAM;

    rclcpp::Node::SharedPtr node;
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

    cloud.header.stamp = node->get_clock()->now();
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
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,false);

    ImageGrabber igb(&SLAM);


    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub(igb.node,"/cam0/image_raw",rclcpp::SensorDataQoS().get_rmw_qos_profile());
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub(igb.node,"/cam1/image_raw",rclcpp::SensorDataQoS().get_rmw_qos_profile());

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(std::bind(&ImageGrabber::GrabStereo,&igb,std::placeholders::_1,std::placeholders::_2));

    rclcpp::spin(igb.node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
//    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
//    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,const sensor_msgs::msg::Image::ConstSharedPtr msgRight)
{


//    clock_t start, finish;
//    double duration;
//    start = clock();
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        //RCLCPP_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        //RCL_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    Sophus::SE3f Tcw_SE3F=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,rclcpp::Time(cv_ptrLeft->header.stamp).seconds());

//    finish = clock();
//    duration = (double)(finish - start) / CLOCKS_PER_SEC;
//    cout<<"The run time is:"<<duration<<endl;

        //1.显示位姿，以TF方式发布出去 map->camera_link
        cv::Mat Tcw;
        Eigen::Matrix4f Tcw_Matrix = Tcw_SE3F.matrix();
        cv::eigen2cv(Tcw_Matrix, Tcw);

        tf2::Transform tf_transform = TransformFromMat(Tcw);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "camera_link";
        tf_msg.header.stamp=node->get_clock()->now();
        tf_msg.transform = tf2::toMsg(tf_transform);

        tf2_ros::TransformBroadcaster tf_broadcaster(node);
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

