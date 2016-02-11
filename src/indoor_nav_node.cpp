#include <ros/ros.h>
#include <indoor_nav/Errors.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>

enum states {
	NO_MISSION,
	MISSION,
} state;

double quaternionToYaw(geometry_msgs::Quaternion quat) {
 	double siny_cosp = +2.0 * (quat.w * quat.z + quat.x * quat.y);
 	double cosy_cosp = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
 	return atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "indoor_nav_node");
	ros::NodeHandle n("~");
	ros::Publisher pub_errors = n.advertise<indoor_nav::Errors>("/errors", 1);
	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);
	ros::Rate loop_rate(20);

	double x_goal = 0, y_goal = 0, theta_goal = 0;
	indoor_nav::Errors errors_msg;
	errors_msg.distance_error = 0;
	errors_msg.angle_error = 0;
  	geometry_msgs::TransformStamped transformStamped;
  	geometry_msgs::Vector3 translation;
  	geometry_msgs::Quaternion quat;
	while(ros::ok()) {
		ros::spinOnce();

		try {
  			transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
 		}

		catch (tf2::TransformException &ex) {
   			//ROS_WARN("%s",ex.what());
   			continue;
 		}

		if(state == MISSION) {
				translation = transformStamped.transform.translation;
				quat = transformStamped.transform.rotation;

				errors_msg.distance_error = sqrt(pow((x_goal - translation.x),2.0) + pow((y_goal - translation.y),2.0));

				theta_goal = atan2(y_goal - translation.y, x_goal - translation.x);
				errors_msg.angle_error = theta_goal - quaternionToYaw(quat);

				if(errors_msg.angle_error > M_PI) errors_msg.angle_error = errors_msg.angle_error - (2 * M_PI);
				if(errors_msg.angle_error < -M_PI) errors_msg.angle_error = errors_msg.angle_error + (2 * M_PI);

				if(errors_msg.distance_error < 0.2) {
					errors_msg.angle_error = 0;
					errors_msg.distance_error = 0;
					state = NO_MISSION;
				}
		}

		pub_errors.publish(errors_msg);

		if(state == NO_MISSION) {
			std::cout << "Introduce the desired goal in 'X Y':"  << std::endl;
			std::cin >> x_goal >> y_goal;
			state = MISSION;
		}
	
	loop_rate.sleep();
	}

	ros::shutdown();
}
