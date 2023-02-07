#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

class FuseCloudsNode
{
    public:
    FuseCloudsNode(string topic1, string topic2, string topic3);
    FuseCloudsNode(FuseCloudsNode &&) = default;
    FuseCloudsNode(const FuseCloudsNode &) = default;
    FuseCloudsNode &operator=(FuseCloudsNode &&) = default;
    FuseCloudsNode &operator=(const FuseCloudsNode &) = default;

    private:
    ros::NodeHandle nh_;
    std::string t1, t2, t3;
    ros::Publisher pub_;
    message_filters::Subscriber<PointCloud2> t1_sub_, t2_sub_, t3_sub_;
    TimeSynchronizer<PointCloud2, PointCloud2, PointCloud2> sync_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_;

    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud1, const sensor_msgs::PointCloud2::ConstPtr& cloud2, const sensor_msgs::PointCloud2::ConstPtr& cloud3);
};

FuseCloudsNode::FuseCloudsNode(string topic1, string topic2, string topic3):
t1(topic1),
t2(topic2),
t3(topic3),
t1_sub_(nh_, t1, 1),
t2_sub_(nh_, t2, 1),
t3_sub_(nh_, t3, 1),
sync_(t1_sub_, t2_sub_, t3_sub_, 10)
{
    sync_.registerCallback(boost::bind(&FuseCloudsNode::callback,this, _1, _2, _3));
    pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("output", 1);
}

void FuseCloudsNode::callback(const sensor_msgs::PointCloud2::ConstPtr& msg1, const sensor_msgs::PointCloud2::ConstPtr& msg2, const sensor_msgs::PointCloud2::ConstPtr& msg3)
{
    // get clouds in pcl
    pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2, cloud3, merge;
    pcl::fromROSMsg(*msg1, cloud1);
    pcl::fromROSMsg(*msg2, cloud2);
    pcl::fromROSMsg(*msg3, cloud3);
    // TODO: transform clouds 
    // Dont need LMAO
    // 
    // merge them
    merge =  cloud1;
    merge += cloud2;
    merge += cloud3;
    // publish merged pointcloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(merge, output);
    pub_.publish(output);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fuse_clouds_node");
    FuseCloudsNode node = FuseCloudsNode("/forward_center_fused", "/left_fused", "/right_fused");
    ros::spin();
    ros::shutdown();
    return 0;
}
