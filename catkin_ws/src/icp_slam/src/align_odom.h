#include <iostream>
#include <vector>
#include <Eigen/Dense>
// Ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace ros;
using namespace std;
const double PI  =3.141592653589793238463;

class align_odom{
  public:
	align_odom();
	void process_node(const nav_msgs::Odometry msg);    // sensor_msgs::PointCloud2
  private:
  	int count;
  	Publisher odom_pub;
	Subscriber odom_sub;
	Eigen::Matrix4f initial_state;
};