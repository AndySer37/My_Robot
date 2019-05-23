#include "align_odom.h"

align_odom::align_odom(){
	NodeHandle nh;

	count = 0;
	odom_pub = nh.advertise<nav_msgs::Odometry> ("/husky/transfer_odom", 10);
	odom_sub = nh.subscribe("/husky_velocity_controller/odom", 1, &align_odom::process_node, this);
}

void align_odom::process_node(const nav_msgs::Odometry msg){
	Eigen::Quaternionf q1(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z); 
	Eigen::Matrix3f rot = q1.toRotationMatrix();
	Eigen::Vector3f trans;
	trans << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ;
	Eigen::Matrix4f state;
	state << rot(0,0), rot(0,1), rot(0,2), trans(0),\
					rot(1,0), rot(1,1), rot(1,2), trans(1),\
					rot(2,0), rot(2,1), rot(2,2), trans(2),\
					0, 0, 0, 1;
	if (count == 0){
		initial_state = state.inverse();
	}
	state = initial_state * state;

	Eigen::Matrix3f mat3x3;
	mat3x3 << state(0,0), state(0,1), state(0,2),\
				state(1,0), state(1,1), state(1,2),\
				state(2,0), state(2,1), state(2,2);

	nav_msgs::Odometry nav;
	nav.header = msg.header;
	nav.child_frame_id = "base_link";
	Eigen::Quaternionf q2(mat3x3); 
	nav.pose.pose.position.x = state(0,3);
	nav.pose.pose.position.y = state(1,3);
	nav.pose.pose.position.z = state(2,3);
	nav.pose.pose.orientation.w = q2.w();
	nav.pose.pose.orientation.x = q2.x();
	nav.pose.pose.orientation.y = q2.y();
	nav.pose.pose.orientation.z = q2.z();
	for(int i = 0 ; i < 6; i++){
		nav.pose.covariance[i + i*6] = 0.001;
	}
	nav.pose.covariance[35] = 0.03;

	odom_pub.publish(nav);

	count ++;
}
int main(int argc, char** argv){
	init(argc, argv, "align_odom");
	align_odom align_odom;
	spin();
	return 0;
}