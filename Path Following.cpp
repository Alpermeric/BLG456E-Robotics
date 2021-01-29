//***************************************
//***
//*** BLG 456E - ROBOTICS - Homework 2
//*** Ihsan SOYDEMIR - 150180702
//*** Alper Meric - 150160714
//*** Bonus-1: Please refer line 184 for P controller
//*** Bonus-2: Robot conquerors route-3 and route-4
//***
//***************************************
#include "ros/ros.h"
#include <iostream>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <math.h>
//contains waypoint data
geometry_msgs::Transform waypoint;

//the data structure that will receive the current pose
//the "stamped" means simply that there is time-stamp information available in the data structure's fields
tf::StampedTransform robot_pose;

//for containing the motor commands to send to the robot
geometry_msgs::Twist motor_command;

gazebo_msgs::GetModelState getModelState ;

//waypoint callback
void waypoint_callback(const geometry_msgs::Transform::ConstPtr& msg) // <--- callback
{

    //***************************************
    //***          Obtain current destination
    //***************************************

    //save waypoint data for printing out in main loop
    waypoint=*msg;

}

int main(int argc, char **argv)
{
	std::string modelName = (std::string)"mobile_base" ;
	std::string relativeEntityName = (std::string)"world" ;
	
    //setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    ros::init(argc, argv, "crazy_driver_456");
    ros::NodeHandle node;
    ros::Subscriber waypoint_subscriber = node.subscribe("/waypoint_cmd", 1000,waypoint_callback); // <--- set up callback
    ros::Publisher motor_command_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
    ros::ServiceClient client;
    ros::NodeHandle other_node;
    // Create service client to retrieve model state from Gazebo
    client = other_node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
    getModelState.request.model_name = modelName; //"mobile_base"
    getModelState.request.relative_entity_name = relativeEntityName; //"world"
    //you could in principle also subscribe to the laser scan as is done in assignment 1.

    //setup transform cache manager
    tf::TransformListener listener;

    //start a loop; one loop per second
    ros::Rate delay(10.0); // perhaps this could be faster for a controller?
    while(ros::ok()){

        //***************************************
        //***          Obtain current robot pose
        //***************************************

        ros::spinOnce(); // may be needed to call the callback

        try{
            //listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(0.001));
            listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), robot_pose);
        }
            //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        //***************************************
        //***          Print current robot pose
        //***************************************

        //Print out the x,y coordinates of the transform
        std::cout<<"Robot is believed to be at (x,y): ("<<robot_pose.getOrigin().x()<<","<<robot_pose.getOrigin().y()<<")"<<std::endl;

        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Vector3 robot_axis=robot_pose.getRotation().getAxis();
        double robot_theta=robot_pose.getRotation().getAngle()*robot_axis[2]; // only need the z axis
        std::cout<<"Robot is believed to have orientation (theta): ("<<robot_theta<<")"<<std::endl<<std::endl;

        //***************************************
        //***          Print current destination
        //***************************************

        // the curr_waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        // subscribed to in the .subscribe function call above.

        //Print out the x,y coordinates of the latest message
        std::cout<<"Current waypoint (x,y): ("<<waypoint.translation.x<<","<<waypoint.translation.y<<")"<<std::endl;

        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        tf::Quaternion quat(waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w);
        tf::Vector3 waypoint_axis=quat.getAxis();
        double waypoint_theta=quat.getAngle()*waypoint_axis[2]; // only need the z axis
        std::cout<<"Current waypoint (theta): ("<<waypoint_theta<<")"<<std::endl<<std::endl;
		
		//*************************************************************************
        //*** Get Modelstate Information for Turtlebot from Gazebo
        //*************************************************************************

		geometry_msgs::Quaternion orientations;
		double pos_x = 0.0;
        double pos_y = 0.0;

        if ( client.call(getModelState) ) { //Get model state
            pos_x = getModelState.response.pose.position.x ;
            pos_y = getModelState.response.pose.position.y ;
            orientations = getModelState.response.pose.orientation ;
		
        }
        else{
            ROS_ERROR("Failed to retrieve model state");

        }

		tf::Quaternion tf_quat(orientations.x,orientations.y,orientations.z,orientations.w);

		double gazebo_service_ga,gazebo_service_gb,gazebo_service_theta;
		tf::Matrix3x3(tf_quat).getRPY(gazebo_service_ga,gazebo_service_gb,gazebo_service_theta);

		tf::Transform transform;
		tf::Quaternion q;
        tf::TransformBroadcaster broadcaster; // Broadcast transform

        //***************************************
        //*** Create new fix frame for waypoints
        //***************************************

		transform.setOrigin( tf::Vector3(waypoint.translation.x, waypoint.translation.y, 0) ); 
		q.setRPY(0, 0, waypoint_theta);
        transform.setRotation(q);
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom","/waypoint"));
		
        //***************************************
        //*** Create a better base_footprint
        //***************************************

		transform.setOrigin( tf::Vector3(pos_x, pos_y, 0) ); 
		q.setRPY(0, 0, gazebo_service_theta);
        transform.setRotation(q);
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom","/better_base_footprint"));		

        //********************************************************
        //*** Obtain new waypoints using better_base_footprint
        //********************************************************

        try{
            //listener.waitForTransform("/better_base_footprint", "/waypoint", ros::Time(0), ros::Duration(0.001));
            listener.lookupTransform("/better_base_footprint","/waypoint", ros::Time(0), robot_pose);
        }
            //if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        double robot_to_waypoint_x = robot_pose.getOrigin().x();
        double robot_to_waypoint_y = robot_pose.getOrigin().y();
        
        double roll, pitch, yaw;
        quat = robot_pose.getRotation();
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		double robot_to_waypoint_theta = yaw; // only need the z axis
		
        //************************************************
        //*** DRIVING THE ROBOT VIA P CONTROLLER
        //************************************************
        
        //ROS_INFO("Waypoint X: %f, Waypoint Y %f", robot_to_waypoint_x, robot_to_waypoint_y);
        //ROS_INFO("ModelPos X: %f, ModelPos Y %f", pos_x, posy);

        // Angular speed calculated with arc tangent
        double angular_velo = 0.85* atan2(robot_to_waypoint_y,robot_to_waypoint_x); 
        // Linear speed calculated using sqrt(a^2+b^2)
        double linear_velo  = 0.95* hypot(robot_to_waypoint_y,robot_to_waypoint_x);

        // If robot is very close to the waypoint goal then only fix orientation
		if (fabs(robot_to_waypoint_x) < 0.1 && fabs(robot_to_waypoint_y) < 0.1){
			motor_command.angular.z = robot_to_waypoint_theta / 2.0; // Turn around
        	motor_command.linear.x = 0.0; // No linear speed needed
            ROS_INFO("I'm very close to the waypoint");
		}else{
            // If waypoint is located at robot's back
			if (robot_to_waypoint_x < 0){ 
				motor_command.angular.z = - angular_velo / 3.0;
				motor_command.linear.x = -linear_velo; //Go backwards
                ROS_INFO("Go backwards");
			}
            // General case
			else{
				motor_command.angular.z = angular_velo;
				motor_command.linear.x = linear_velo;
                ROS_INFO("Go towards waypoint");
			}
		}
        motor_command_publisher.publish(motor_command);
        delay.sleep();
    }
    return 0;
}
