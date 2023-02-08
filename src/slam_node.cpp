#include <string>
#include <vector>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

// GTSAM
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose3.h>

void print4x4Matrix (const Eigen::Matrix4d & matrix){
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

class SlamNode
{
public:
    SlamNode();
    SlamNode(SlamNode &&) = default;
    SlamNode(const SlamNode &) = default;
    SlamNode &operator=(SlamNode &&) = default;
    SlamNode &operator=(const SlamNode &) = default;

private:
    bool first_scan_;
    double min_scan_range_, max_scan_range_;
    int count_;

    ros::NodeHandle nh_;
    ros::Subscriber lidar_Sub_;
    ros::Publisher map_pub_, source_pc_pub_, target_pc_pub_;

    gtsam::Pose3 last_pose_;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB, double> icp_;
    pcl::PointCloud<pcl::PointXYZRGB> map_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> map_list_;

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void removeDynObj(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    pcl::PointCloud<pcl::PointXYZRGB> removeGround(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    Eigen::Matrix4d transform2D(double theta, double xt, double yt);
};

SlamNode::SlamNode() :
first_scan_(true),
min_scan_range_(10.0),
max_scan_range_(300.0),
count_(0)
{
    // ROS
    lidar_Sub_ = nh_.subscribe("output", 1, &SlamNode::pointcloudCallback, this);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map", 1);
    source_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("source_pc_", 1);
    target_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("target_pc_", 1);

    // ICP
    icp_.setMaxCorrespondenceDistance(2.0);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setMaximumIterations(50);
}

void SlamNode::removeDynObj(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    std::cout << "removing things...." << std::endl;
    double r;
    pcl::PointXYZRGB p;
    pcl::PointCloud<pcl::PointXYZRGB> final;
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator item = cloud.begin(); item != cloud.end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.r = (uint8_t)item->r;
        p.g = (uint8_t)item->g;
        p.b = (uint8_t)item->b;

        r = sqrt(p.x*p.x + p.y*p.y);
        if (min_scan_range_ > r)
        {
            continue;
        }
        if ((p.r == 0) && (p.g == 0) && (p.b == 142)) // vehicles
        {
            continue;
        }
        else if ((p.r == 220) && (p.g == 20) && (p.b == 60))
        {
            continue;
        }
        final.push_back(p);
    }
    cloud = final;
}

pcl::PointCloud<pcl::PointXYZRGB> SlamNode::removeGround(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PointXYZRGB p;
    pcl::PointCloud<pcl::PointXYZRGB> final;
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator item = cloud.begin(); item != cloud.end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.r = (uint8_t)item->r;
        p.g = (uint8_t)item->g;
        p.b = (uint8_t)item->b;

        if (p.z >= 0.5)
        {
            final.push_back(p);
        }
    }
    return final;
}

Eigen::Matrix4d SlamNode::transform2D(double theta, double xt, double yt)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

    matrix(0, 3) = xt;
    matrix(1, 3) = yt;

    matrix(0, 0) = cos(theta);
    matrix(0, 1) = -sin(theta);
    matrix(1, 0) = sin(theta);
    matrix(1, 1) = cos(theta);

    return matrix;
}

void SlamNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> in_cloud_;
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    pcl::fromROSMsg(*msg,   in_cloud_);
    removeDynObj(in_cloud_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>(removeGround(in_cloud_)));

    std::cout << "count_ = " << count_++ << std::endl;
    if (first_scan_)
    {
        gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
        // initial_estimate_.insert(0, pose);
        last_pose_ = pose;
        first_scan_ = false;
        map_list_.push_back(in_cloud_);
    }
    else
    {
        // get last transformation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>(map_list_.back()));
        pcl::PointCloud<pcl::PointXYZRGB> final;
        icp_.setInputSource(source);
        icp_.setInputTarget(target);
        icp_.align(final);
        if (icp_.hasConverged()) {
            std::cout << "has converged: " << icp_.hasConverged() << " score: " << icp_.getFitnessScore() << std::endl;
            transformation_matrix = icp_.getFinalTransformation().cast<double>();
            print4x4Matrix(transformation_matrix);

            gtsam::Pose3 new_pose = last_pose_.compose(gtsam::Pose3(gtsam::Rot3(transformation_matrix.block<3,3>(0, 0)),
                gtsam::Point3(transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3))));
            last_pose_ = new_pose;
            // publish compiled map
            Eigen::Matrix4d tf = last_pose_.matrix();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud (in_cloud_, *transformed_source, tf);
            map_ += *transformed_source;
            map_list_.push_back(in_cloud_);

            sensor_msgs::PointCloud2 output, source_pc_, target_pc_;
            pcl::toROSMsg(map_, output);
            pcl::toROSMsg(*transformed_source, source_pc_);
            pcl::toROSMsg(*target, target_pc_);
            output.header.frame_id = msg->header.frame_id;
            output.header.stamp = msg->header.stamp;
            source_pc_.header.frame_id = msg->header.frame_id;
            source_pc_.header.stamp = msg->header.stamp;
            target_pc_.header.frame_id = msg->header.frame_id;
            target_pc_.header.stamp = msg->header.stamp;
            map_pub_.publish(output);
            source_pc_pub_.publish(source_pc_);
            target_pc_pub_.publish(target_pc_);

            transformation_matrix = last_pose_.matrix();
            std::cout << "x= " << last_pose_.x() << " y= "<< last_pose_.y() << " z= " << last_pose_.z() << std::endl
                      << "roll= " << last_pose_.rotation().roll() << " pitch= " << last_pose_.rotation().pitch() << " yaw= " << last_pose_.rotation().yaw() << std::endl; 

            /*
            // Get transformation from ICP result and update pose estimate
            Eigen::Matrix4f transformation = icp_.getFinalTransformation();
            gtsam::Pose3 new_pose = last_pose_.compose(gtsam::Pose3(gtsam::Rot3::RzRyRx(
            transformation(2, 0), transformation(1, 0), transformation(0, 0)),
            gtsam::Point3(transformation(0, 3), transformation(1, 3), transformation(2, 3))));
            initial_estimate_.insert(initisal_estimate_.size(), new_pose);
            factor_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(
            initial_estimate_.size() - 2, initial_estimate_.size() - 1, last_pose_.between(new_pose),
            gtsam::noiseModel::Isotropic::Variance(6, 1e-6));

            // Update last pose
            last_pose_ = new_pose;
            */
        }
        else    return;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_node");
    SlamNode node = SlamNode();
    ros::spin();
    ros::shutdown();
    return 0;
}