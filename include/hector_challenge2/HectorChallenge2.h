#ifndef HECTORCHALLENGE2_H_
#define HECTORCHALLENGE2_H_

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "gazebo_msgs/SetModelState.h"
#include <gazebo_msgs/ModelStates.h>
#include <hector_uav_msgs/EnableMotors.h>





namespace hector_challenge2
{
	class HectorChallenge2
	{
		public:
			// constructor
			HectorChallenge2(ros::NodeHandle& n);
			
			// destructor
			virtual ~HectorChallenge2();
		private:

		//create Ros nodehandle, sub, and pub 
			ros::NodeHandle& n_;
			ros::Subscriber sub_IMU;
			ros::Subscriber sub_pos;
			ros::Subscriber  sub_state;
			ros::Publisher  pub_;
			ros::ServiceClient client;
			ros::ServiceClient client_motor;
			
			
		
			
		//create methods		
	  void imuCallback(const sensor_msgs::Imu &msg3);	
	  void posCallback(const geometry_msgs::PoseStamped &msg4);
	  void ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg_pos);
	  void PID(); 
	  void Path();
	  void objectmotion();
	  void motor_enable();
	  bool Parameters();
	  void twist();

          //create arguments
		  geometry_msgs::Twist msg2;
		  
		  
			float ref_angle;
			float smallest_distance;
			float piller_x,piller_y,piller_z;
			double roll,yaw,pitch;
			float roll_dot,yaw_dot,pitch_dot;
			float vx,vy,vz;
			float vxi,vyi,vzi;
			float pos_x,pos_y,pos_z; 
			float qaut_x,qaut_y,qaut_z,qaut_w;
			float pos_prev_x,pos_prev_y,pos_prev_z;
			float error_x,error_y,error_z,error=0.6;
			float error_d_x,error_d_y,error_d_z;
			float error_i_x,error_i_y,error_i_z;
			float t,t_prev;
			float pos_targ_x;
			float pos_targ_y;
			float pos_targ_z;
			float kp_x,kp_y,kp_z;
			float ki_x,ki_y,ki_z;
			float kd_x,kd_y,kd_z;
			double x,y,r;
			double dt;
			std::string object_name;
			int count_path=0;
			int count=0,count_object=1,object_size=27;
			double theta=0.0;
            std::vector< double > pathx;
            std::vector< double > pathy;
			std::vector< double > pathz;
			std::vector< double > pathx_target;
            std::vector< double > pathy_target;
			std::vector< double > pathz_target;

			
			 
			 
	};
	
}




#endif