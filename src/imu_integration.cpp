#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

class ImuIntegration
{
public:
    ImuIntegration();
    ImuIntegration(ImuIntegration &&) = default;
    ImuIntegration(const ImuIntegration &) = default;
    ImuIntegration &operator=(ImuIntegration &&) = default;
    ImuIntegration &operator=(const ImuIntegration &) = default;
    ~ImuIntegration();

private:
    Eigen::Vector3d bias_accel_ = Eigen::Vector3d::Zero(9);
    Eigen::Vector3d bias_gyro_ = Eigen::Vector3d::Zero(3);

    Eigen::VectorXd observation_ = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd state_ = Eigen::VectorXd::Zero(9);
    Eigen::Vector3d accel_obs_ = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d gyro_obs_ = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d accel_inertial_ = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d gn_ = Eigen::Vector3d::Zero(3);

    Eigen::Matrix3d rotation_;
    Eigen::Matrix3d rotation_rate_;

    double dt_ = 0;

    Eigen::Matrix4d rotation3D(double yaw, double pitch, double roll);
    Eigen::Matrix4d transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt);
    void imuCallback(const sensor_msgs::Imu::Ptr msg);
    void integration();
};

ImuIntegration::ImuIntegration()
{
    observation_ = Eigen::VectorXd::Zero(9); // nine integration states
    state_ = Eigen::VectorXd::Zero(9);
}

Eigen::Matrix4d ImuIntegration::rotation3D(double yaw, double pitch, double roll)
{
    Eigen::Matrix4d matrix = Eigen::Matrix3d::Identity ();

    matrix(0, 0) = cos(yaw) * cos(pitch);
    matrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    matrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
    matrix(1, 0) = sin(yaw) * cos(pitch);
    matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    matrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
    matrix(2, 0) = -sin(pitch);
    matrix(2, 1) = cos(pitch) * sin(roll);
    matrix(2, 2) = cos(pitch) * cos(roll);

    return matrix;
}

Eigen::Matrix4d ImuIntegration::transform3D(double yaw, double pitch, double roll, double xt, double yt, double zt) {

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

    matrix(0, 3) = xt;
    matrix(1, 3) = yt;
    matrix(2, 3) = zt;

    matrix.block<3, 3>(0, 0) = rotation3D(yaw, pitch, roll);

    return matrix;
}

void ImuIntegration::imuCallback(const sensor_msgs::Imu::Ptr msg)
{
    accel_obs_(0) = msg->linear_acceleration.x;
    accel_obs_(1) = msg->linear_acceleration.y;
    accel_obs_(2) = msg->linear_acceleration.z;

    gyro_obs_(0) = msg->angular_velocity.x;
    gyro_obs_(1) = msg->angular_velocity.y;
    gyro_obs_(2) = msg->angular_velocity.z;

    rotation_ = rotation3D(state_(8), state_(7), state_(6));
    rotation_rate_ << gyro_obs_(0), 0, 0,
                      0, gyro_obs_(1), 0,
                      0, 0, gyro_obs_(2);
}

void ImuIntegration::integration()
{
    if(dt_ > 0)
    {
        accel_inertial_ = rotation_ * (accel_obs_ - bias_accel_);
        state_.block<3, 0>(0, 0) = state_.block<3, 0>(0, 0) + state_.block<3, 0>(3, 0) * dt_; // position
        state_.block<3, 0>(3, 0) = state_.block<3, 0>(3, 0) + accel_inertial_ * dt_ + gn_ * dt_; // velocity
        state_.block<3, 0>(6, 0) = state_.block<3, 0>(6, 0) + rotation_rate_ * (gyro_obs_ - bias_gyro_) * dt_; // angles
    }
}

ImuIntegration::~ImuIntegration()
{
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "imu_integration");
    auto imu_integration = /*namespace_name::*/ImuIntegration();
    ros::spin();
    ros::shutdown();
    return 0;
}
