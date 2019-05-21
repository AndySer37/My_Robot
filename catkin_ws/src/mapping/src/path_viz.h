#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

using namespace ros;
using namespace std;

class path_viz{
 public:
  path_viz(NodeHandle& );
  void path_cb();
 private:
  Subscriber sub;
  Publisher marker_pub;
  uint32_t shape;
  visualization_msgs::Marker marker;
  tf::TransformListener listener;
};
