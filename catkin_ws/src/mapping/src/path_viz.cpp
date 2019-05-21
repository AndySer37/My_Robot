#include"path_viz.h"

path_viz::path_viz(NodeHandle& nh){    
    shape = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = "/slam_map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    while(nh.ok()){
        path_cb();
        ros::Duration(0.5).sleep();
    }

}
void path_viz::path_cb(){

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/slam_map", "/scanmatcher_frame",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        return;
    }

    geometry_msgs::Point p;
    p.x = transform.getOrigin().x();
    p.y = transform.getOrigin().y();
    p.z = transform.getOrigin().z();

    marker.points.push_back(p);
    marker.lifetime = ros::Duration(1);
    marker_pub.publish(marker);
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "path_viz");
  ros::NodeHandle nh;
  path_viz path_viz(nh);
  ros::spin(); 
}