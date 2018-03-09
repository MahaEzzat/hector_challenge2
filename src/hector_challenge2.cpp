#include <ros/ros.h>
#include "hector_challenge2/HectorChallenge2.h"

namespace hector_challenge2
{
    HectorChallenge2::HectorChallenge2(ros::NodeHandle& n) :
    n_(n)
    {

if(!Parameters())
        {
            ROS_ERROR("Error in loading parameters");  
            ros::shutdown();
        }

      HectorChallenge2::motor_enable();
      HectorChallenge2::Path();
      sub_state = n_.subscribe("/gazebo/model_states", 1, &HectorChallenge2::ModelStatecallback,this);  
      sub_pos = n_.subscribe("/ground_truth_to_tf/pose",1,&HectorChallenge2::posCallback,this); 
      sub_IMU = n_.subscribe("/raw_imu",1,&HectorChallenge2::imuCallback,this);  
      client = n_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
      pub_=  n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);   

    }

    HectorChallenge2::~HectorChallenge2()
    {

    }



//Methods  

void HectorChallenge2::Path()
{

//Object initail position
 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(3);

 pathx.push_back(0.0);
 pathy.push_back(0.0);
 pathz.push_back(0.35);

//object final position
 pathx.push_back(0);
 pathy.push_back(0);
 pathz.push_back(3);

 pathx.push_back(0);
 pathy.push_back(0);
 pathz.push_back(0.5);
 
 //target_points
  for(int i=0;i<object_size-2;i++)
 {   
    pathx_target.push_back(0.0);
    pathy_target.push_back(-0.71041);
    pathz_target.push_back(0.4+i*0.101);

    pathx_target.push_back(0.0);
    pathy_target.push_back(0.7105);
    pathz_target.push_back(0.4+i*0.101);

    pathx_target.push_back(0.0);
    pathy_target.push_back(-0.238);
    pathz_target.push_back(0.4+i*0.101);

    pathx_target.push_back(0.0);
    pathy_target.push_back(0.238);
    pathz_target.push_back(0.4+i*0.101);
 }

}


    void HectorChallenge2::ModelStatecallback(const gazebo_msgs::ModelStates::ConstPtr& msg_pos)
{ 
  pathx[0]= msg_pos->pose[count_object].position.x;
  pathy[0]= msg_pos->pose[count_object].position.y;
  

  pathx[1]= msg_pos->pose[count_object].position.x;
  pathy[1]= msg_pos->pose[count_object].position.y;
  
  
  pathx[2]= pathx_target[count_object-1];
  pathy[2]= pathy_target[count_object-1];
  

  pathx[3]= pathx_target[count_object-1];
  pathy[3]= pathy_target[count_object-1];
  pathz[3]= pathz_target[count_object-1];



  object_name = msg_pos->name[count_object];
  object_size = msg_pos->name.size();

}


 void HectorChallenge2::imuCallback(const sensor_msgs::Imu &msg3)
    {
 tf::Quaternion bg(msg3.orientation.x,msg3.orientation.y,msg3.orientation.z,msg3.orientation.w);
 tf::Matrix3x3(bg).getRPY(roll,pitch,yaw);
 qaut_x = msg3.orientation.x;
 qaut_y = msg3.orientation.y;
 qaut_z = msg3.orientation.z;
 qaut_w = msg3.orientation.w;
 if(count_path>1) HectorChallenge2::objectmotion();
 
 HectorChallenge2::PID();
 HectorChallenge2::twist();
    }


void HectorChallenge2::posCallback(const geometry_msgs::PoseStamped &msg4)
  {
  pos_x =  msg4.pose.position.x;
  pos_y =  msg4.pose.position.y;
  pos_z =  msg4.pose.position.z;
  }


void HectorChallenge2::twist()
    {
  msg2.linear.x = vxi;
  msg2.linear.y = vyi;
  msg2.linear.z = vzi;
  msg2.angular.x = 0;
  msg2.angular.y = 0; 
  msg2.angular.z = 0; 
  pub_.publish(msg2);
    }




void HectorChallenge2::PID()
{
pos_targ_x = pathx[count_path];
pos_targ_y = pathy[count_path];
pos_targ_z = pathz[count_path];

if(count==0)
{
t_prev = ros::Time::now().toSec();
  error_d_x =0;
  error_d_y =0;
  error_d_z =0;
  error_i_x = 0;
  error_i_y = 0;
  error_i_z = 0;
  error=0.6;
  count++;
}

else
{ 
t = ros::Time::now().toSec() ;
dt = t - t_prev;
t_prev = t;

error_d_x = (pos_prev_x-pos_x)/dt;
error_d_y = (pos_prev_y-pos_y)/dt;
error_d_z = (pos_prev_z-pos_z)/dt;

error_i_x += error_x*dt;
error_i_y += error_y*dt;
error_i_z += error_z*dt;
}

error_x = pos_targ_x-pos_x;
error_y = pos_targ_y-pos_y;
error_z = pos_targ_z-pos_z;
error = sqrt(error_x*error_x+error_y*error_y+error_z*error_z);


vx = kp_x*error_x + ki_x * error_i_x + kd_x *error_d_x;
vy = kp_y*error_y + ki_y * error_i_y + kd_y *error_d_y;
vz = kp_z*error_z + ki_z * error_i_z + kd_z *error_d_z;



if( error < 0.02) {
    count_path++;
    ROS_INFO("count_path: [%d]",count_path);
    if(count_path>(pathy.size()-1))
    { count_path=0;
      count_object++;
      if(count_object>object_size-2) count_object=1;
    }
    ROS_INFO("count_path: [%d]", count_path);
}

pos_prev_x = pos_x;
pos_prev_y = pos_y;
pos_prev_z = pos_z;



vxi = (cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch))*vx - cos(roll)*sin(yaw)*vy + (cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw))*vz;
vyi = (sin(yaw)*cos(pitch)+sin(roll)*cos(yaw)*sin(pitch))*vx + cos(roll)*cos(yaw)*vy + (sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw))*vz;
vzi = -cos(roll)*sin(pitch)*vx  +  sin(roll)*vy   + cos(pitch)*cos(roll)*vz;
}




void HectorChallenge2::objectmotion()

{
    // Position
    geometry_msgs::Point pr2_position;
    pr2_position.x = pos_x;
    pr2_position.y = pos_y;
    pr2_position.z = pos_z-0.35;

    // orientation
    geometry_msgs::Quaternion pr2_orientation;
    pr2_orientation.x = qaut_x;
    pr2_orientation.y = qaut_y;
    pr2_orientation.z = 0.0;
    pr2_orientation.w = qaut_w;

    // pose (Pose + Orientation)
    geometry_msgs::Pose pr2_pose;
    pr2_pose.position = pr2_position;
    pr2_pose.orientation = pr2_orientation;

    //ModelState
    gazebo_msgs::ModelState pr2_modelstate;
    pr2_modelstate.model_name = object_name;
    pr2_modelstate.pose = pr2_pose;



    //gazebo-->pose
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = pr2_modelstate;

    //Server
    if(client.call(srv))
    {
       // ROS_INFO("PR2's magic moving success!!");
    }
    else
    {
       // ROS_ERROR("Failed to magic move PR2! Error msg:%s",srv.response.status_message.c_str());
    }


}



  bool HectorChallenge2::Parameters()
    {
        if(!n_.getParam("kp_x_",kp_x)) return false;
        if(!n_.getParam("kp_y_",kp_y)) return false;
        if(!n_.getParam("kp_z_",kp_z)) return false;
        if(!n_.getParam("ki_x_",ki_x)) return false;
        if(!n_.getParam("ki_y_",ki_y)) return false;
        if(!n_.getParam("ki_z_",ki_z)) return false;
        if(!n_.getParam("kd_x_",kd_x)) return false;
        if(!n_.getParam("kd_y_",kd_y)) return false;
        if(!n_.getParam("kd_z_",kd_z)) return false;
        return true;
    }



void HectorChallenge2::motor_enable()
{
    //motor enable
    client_motor = n_.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    hector_uav_msgs::EnableMotors srv2;
    srv2.request.enable = true;
    if(client_motor.call(srv2))
    {
        ROS_INFO("motor magic moving success!!");
    }
    else
    {
        ROS_ERROR("Failed to magic move motor!");
    }
}

}
