#include <ros/ros.h>
#include "hector_challenge2/HectorChallenge2.h"
int next_object1,next_object2,next_object3;
void object1_subCallback(const std_msgs::Int8 &msg_obj);
void object2_subCallback(const std_msgs::Int8 &msg_obj);
//void object3_subCallback(const std_msgs::Int8 &msg_obj);
ros::Publisher  pub_object1;

int  main(int argc, char **argv)
{
    ros::init(argc, argv, "object");
    ros::NodeHandle n;

 pub_object1 =  n.advertise<std_msgs::Int8>("/object_number",1);
ros::Subscriber sub_object1 = n.subscribe("/q1/object_number",1,object1_subCallback); 
ros::Subscriber sub_object2 = n.subscribe("/q2/object_number",1,object2_subCallback); 
//ros::Subscriber sub_object3 = n.subscribe("/q3/object_number",1,object3_subCallback);
  
    ros::spin();
    return 0;
}


void object1_subCallback(const std_msgs::Int8 &msg_obj)
{
next_object1 = msg_obj.data;
}

void object2_subCallback(const std_msgs::Int8 &msg_obj)
{
next_object2 = msg_obj.data;
std_msgs::Int8 object_index_;
if(next_object1>=next_object2) object_index_.data = next_object1;
if(next_object2>=next_object1) object_index_.data = next_object2;
pub_object1.publish(object_index_);
}

/*void object3_subCallback(const std_msgs::Int8 &msg_obj)
{
next_object3 = msg_obj.data;

std_msgs::Int8 object_index_;
if(next_object1>=next_object2 && next_object1>=next_object3) object_index_.data = next_object1;
if(next_object2>=next_object1 && next_object2>=next_object3) object_index_.data = next_object2;
if(next_object3>=next_object1 && next_object3>=next_object2) object_index_.data = next_object3;

pub_object1.publish(object_index_);

} */
