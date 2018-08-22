#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
//#include <vector.h>

sensor_msgs::Imu imu_data;
geometry_msgs::Vector3 T3_rpy;
geometry_msgs::Vector3 T3_ang_vel;
geometry_msgs::Vector3 T3_lin_acc;
 
//T3_geometry
//Roll
static double r_r_u=63.25;//universal joint center to servo surface length(vertical)
static double r_r_s=80;//universal joint center to servo surface length(horizontal)
static double r_r_h=50;//servo horn length
static double r_r_r=59.33;//push rod length
static double r_r_t=70;//universal joint centor to lower stem length(horizontal)
//Pitch
static double r_p_u=52.25;
static double r_p_s=64.75;
static double r_p_h=50;
static double r_p_r=59.33;
static double r_p_t=120;

static double pi=3.141592;

double theta_r_s=0;//desired roll servo angle 
double theta_p_s=0;//desired pitch servo angle

void imu_update(const sensor_msgs::Imu::ConstPtr &imu){
        geometry_msgs::Quaternion q=imu->orientation;

	T3_rpy.x=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
	double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
	if(fabs(temp_y)>0.9999)	temp_y=(temp_y/fabs(temp_y))*0.9999;
	T3_rpy.y=asin(temp_y);//pitch
	T3_rpy.z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));//yaw
	//Ref: http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html        
	T3_ang_vel=imu->angular_velocity;
        T3_lin_acc=imu->linear_acceleration;
}

void rel_atti_to_servo(double rel_atti_r, double rel_atti_p){
	//Roll
	double r_r_j=sqrt(pow(r_r_u,2)+pow(r_r_s,2));
	double theta_r_1=atan2(r_r_s,r_r_u);
	double theta_r_2=atan2(r_r_u,r_r_s);
	double k_r=sqrt(pow(r_r_j,2)+pow(r_r_t,2)-2*r_r_j*r_r_t*cos(theta_r_2+rel_atti_r));
	double temp_beta_r=((-pow(r_r_t,2)+pow(r_r_j,2)+pow(k_r,2))/(2*r_r_j*k_r));
	if(fabs(temp_beta_r)>0.9999)	temp_beta_r=(temp_beta_r/fabs(temp_beta_r))*0.9999;
	double beta_r=acos(temp_beta_r);
	double temp_alpha_r=((-pow(k_r,2)+pow(r_r_h,2)+pow(r_r_r,2))/(2*r_r_h*r_r_r));
	if(fabs(temp_alpha_r)>0.9999)	temp_alpha_r=(temp_alpha_r/fabs(temp_alpha_r))*0.9999;
	double alpha_r=acos(temp_alpha_r);
	theta_r_s=asin(r_r_r*sin(alpha_r)/k_r)-theta_r_2-beta_r;

	//Pitch
	double r_p_j=sqrt(pow(r_p_u,2)+pow(r_p_s,2));
	double theta_p_1=atan2(r_p_s,r_p_u);
	double theta_p_2=atan2(r_p_u,r_p_s);
	double k_p=sqrt(pow(r_p_t,2)+pow(r_p_j,2)-2*r_p_t*r_p_j*cos(pi/2-theta_p_1-rel_atti_p));
	double temp_alpha_p=((pow(r_p_j,2)+pow(k_p,2)-pow(r_p_t,2))/(2*r_p_j*k_p));
	if(fabs(temp_alpha_p)>0.9999)	temp_alpha_p=(temp_alpha_p/fabs(temp_alpha_p))*0.9999;
	double alpha_p=acos(temp_alpha_p);
	double beta_p=pi-theta_p_2-alpha_p;
	double temp_theta_p_s=(pow(k_p,2)+pow(r_p_h,2)-pow(r_p_r,2))/(2*k_p*r_p_h);
	if(fabs(temp_theta_p_s)>0.9999) temp_theta_p_s=(temp_theta_p_s/fabs(temp_theta_p_s))*0.9999;
	theta_p_s=acos(temp_theta_p_s)-beta_p;
	
}

int main(int argc, char **argv){
        ros::init(argc, argv, "imu_to_servo");
        ros::NodeHandle nh;
        ros::Publisher servo_angle=nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 100);
        ros::Subscriber imu_attitude=nh.subscribe("/imu/data", 100, &imu_update);
        ros::Rate loop_rate(200);
        sensor_msgs::JointState servo_msg;

        while(ros::ok()){
                //ROS_INFO("roll: %f, pitch: %f, yaw: %f", T3_rpy.x, T3_rpy.y, T3_rpy.z);
                //ROS_INFO("roll_s: %f, pitch_s: %f", theta_r_s, theta_p_s);
		
		rel_atti_to_servo(T3_rpy.x, -T3_rpy.y);
		servo_msg.name.resize(2);
		servo_msg.name[0]="id_1";
		servo_msg.name[1]="id_2";
	
		servo_msg.position.resize(2);
		servo_msg.position[0]=theta_r_s;
		servo_msg.position[1]=theta_p_s;
		
		servo_angle.publish(servo_msg);

		ros::spinOnce();
                loop_rate.sleep();
        }

        
}
