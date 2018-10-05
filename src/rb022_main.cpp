#include "/opt/ros/kinetic/include/ros/ros.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#ifndef PI
#define PI 3.1415926
#endif

#define WHEELSPAN_MM 290	//[mm]
#define WHEELSPAN_M 0.29
#define  REDUCTION_RATIO 64
#define  CPR 12	// Counts Per Revolution
#define DEAMETER (144 * PI)	//[mm]


float yaw=0.0f;
uint8_t spd_per=0;
bool imu_ready=0;

sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;	//pose:covarience pose:position orientation	|twist:twist covarience:linear angular
geometry_msgs::PoseStamped pose_msg;	//pose:position orientation
geometry_msgs::TransformStamped tfs_msg;	//transform:translation rotation


ros::Publisher pub_imu;
ros::Publisher pub_odom;
ros::Publisher pub_pose;
tf::TransformBroadcaster tfbc_odom;	

void init(void);
void encCallback(const std_msgs::Int32MultiArray::ConstPtr& array_s32);
void imuCallback(const std_msgs::Float32MultiArray::ConstPtr& array_f32);

int main(int argc,char **argv){
	ros::init(argc,argv,"odom_tf_publisher");
	
	init();
	ros::NodeHandle nh;
	ros::Subscriber subEnc=nh.subscribe("enc_a",1000,encCallback);
	ros::Subscriber subImu=nh.subscribe("imu_a",1000,imuCallback);

	pub_imu=nh.advertise<sensor_msgs::Imu>("imu",50);
	pub_pose=nh.advertise<geometry_msgs::PoseStamped>("pose",50);
	pub_odom=nh.advertise<nav_msgs::Odometry>("odom",50);

	ros::Time current_time,last_time;
	current_time=ros::Time::now();
	
	ros::Rate r(20.0);
	
	ros::spin();
	while(nh.ok()){
		
	}
	return 0;
}		
void init(void){
	imu_msg.header.frame_id="imu_link";
	odom_msg.header.frame_id="odom";
	odom_msg.child_frame_id="base_link";
	odom_msg.pose.pose.position.x=odom_msg.pose.pose.position.y=odom_msg.pose.pose.position.z=0;
}
void encCallback(const std_msgs::Int32MultiArray::ConstPtr& array_s32){
	static int32_t last_millis;
	static float delta_t,vel_l,vel_r;
	static int32_t last_vel[2];
	if(imu_ready){ 
		if(last_vel[0]!=array_s32->data[1] || last_vel[1]!=array_s32->data[2]){
			delta_t=(float)array_s32->data[0]-(float)last_millis;
			last_millis=array_s32->data[0];
			vel_l=(float)array_s32->data[1]/1000;
			vel_r=(float)array_s32->data[2]/1000;
			last_vel[0]=array_s32->data[1];
			last_vel[1]=array_s32->data[2];
			spd_per=(array_s32->data[3]>>4)*10;
		}
		else{
			last_millis=array_s32->data[0];
			vel_l=0;
			vel_r=0;
		}
		odom_msg.pose.pose.position.x+=delta_t*(vel_l+vel_r)/2*cos(yaw);
		odom_msg.pose.pose.position.y+=delta_t*(vel_l+vel_r)/2*cos(yaw);
		
		//odom_msg.pose.pose.orientation=imu_msg.orientation;
		
		//geometry_msgs::Quaternion odom_quat=tf::createQuaternionFromYaw(yaw);
		//odom_msg.pose.pose.orientation=odom_quat;
		
        getQuaternionMsg(0,0,yaw,odom_msg.pose.pose.orientation);
		odom_msg.twist.twist.linear.x=(vel_l+vel_r)/2*cos(yaw);
		odom_msg.twist.twist.linear.y=(vel_l+vel_r)/2*sin(yaw);
		odom_msg.twist.twist.angular.z=imu_msg.angular_velocity.z*PI/180;
		//odom_msg.twist.twist.angular.z=(vel_r-vel_l)/WHEELSPAN_M;
		odom_msg.header.stamp=ros::Time::now();
		pub_odom.publish(odom_msg);
		
		tfs_msg.transform.translation.x=odom_msg.pose.pose.position.x;
        tfs_msg.transform.translation.y=odom_msg.pose.pose.position.y;
        tfs_msg.transform.translation.z=odom_msg.pose.pose.position.z;
        
		tfs_msg.transform.rotation=odom_msg.pose.pose.orientation;
		//tfs_msg.transform.rotation=odom_quat;
		tfs_msg.header.stamp=odom_msg.header.stamp;
		tfbc_odom.sendTransform(tfs_msg);
	}
	
}
void imuCallback(const std_msgs::Float32MultiArray::ConstPtr& array_f32){
	ros::Time curr_time=ros::Time::now();
	imu_msg.linear_acceleration.x=array_f32->data[0];
	imu_msg.linear_acceleration.y=array_f32->data[1];
	imu_msg.linear_acceleration.z=array_f32->data[2];
	imu_msg.angular_velocity.x=array_f32->data[3];
	imu_msg.angular_velocity.y=array_f32->data[4];
	imu_msg.angular_velocity.z=array_f32->data[5];
	imu_msg.orientation.w=array_f32->data[6];
	imu_msg.orientation.x=array_f32->data[7];
	imu_msg.orientation.y=array_f32->data[8];
	imu_msg.orientation.z=array_f32->data[9];
	imu_msg.header.stamp=curr_time;
	yaw=array_f32->data[10];
	yaw=yaw*PI/180;
	pub_imu.publish(imu_msg);
	imu_ready=1;
}
void getQuaternionMsg(
    double roll,double pitch,double yaw,
    geometry_msgs::Quaternion &q
    ){
   tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
   quaternionTFToMsg(quat,q);
}
