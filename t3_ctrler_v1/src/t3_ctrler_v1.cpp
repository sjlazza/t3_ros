#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Vector3.h>

double freq=200;//controller loop frequency

//Sensor_readings=========================================

//RC_readings
int arr[6];//0:roll, 1:pitch, 2:yaw, 3:thrust, 4:3-step switch

//IMU_readings
sensor_msgs::Imu imu_data;
geometry_msgs::Vector3 TGP_rpy;
geometry_msgs::Vector3 TGP_ang_vel;
geometry_msgs::Vector3 TGP_lin_acc;
int yaw_count=0;//counts number of yaw rotation
double yaw_prev=0;//yaw value of previous step

//IMU_NAV_readings
sensor_msgs::Imu imu_nav_data;
geometry_msgs::Vector3 FP_rpy;
geometry_msgs::Vector3 FP_ang_vel;
geometry_msgs::Vector3 FP_lin_acc;
int yaw_nav_count=0;
double yaw_nav_prev=0;
//--------------------------------------------------------


//Commands================================================

//Servo_cmd
double theta_r_s=0;//desired roll servo angle  [DYNAMIXEL XH430-W210-R id-1]
double theta_p_s=0;//desired pitch servo angle [DYNAMIXEL XH430-W210-R id-2]

//Thruster_cmd
double F1=0;//desired propeller 1 force
double F2=0;//desired propeller 2 force
double F3=0;//desired propeller 3 force
double F4=0;//desired propeller 4 force

double e_r_i=0;//roll error integration
double e_p_i=0;//pitch error integration

double tau_r_d=0;//roll  desired torque (N.m)
double tau_p_d=0;//pitch desired torque(N.m)

std_msgs::Int16MultiArray PWMs_cmd;

//ud_cmd
double r_d=0;//desired roll angle
double p_d=0;//desired pitch angle
double y_d=0;//desired yaw angle
double y_d_tangent=0;//yaw increment tangent
double T_d=0;//desired thrust

//--------------------------------------------------------


//T3_V2_geometry==========================================

//General dimensions
static double l_arm=0.2535;//(m), [motor center] to [fp center] length
static double mass=3.2;//(Kg)

//Servo mechnism-Roll(mm)
static double r_r_u=63.25;//[u-joint center] to [servo surface] length(vertical)
static double r_r_s=80;//[u-joint center] to [servo surface] length(horizontal)
static double r_r_h=50;//[servo horn] length
static double r_r_r=59.33;//[push rod] length
static double r_r_t=70;//[u-joint centor] to [lower stem] length(horizontal)

//Servo mechanism-Pitch(mm)
static double r_p_u=52.25;
static double r_p_s=64.75;
static double r_p_h=50;
static double r_p_r=59.33;
static double r_p_t=120;

//Propeller constants(DJI E800(3510 motors + 620S ESCs))

static double b_over_k_ratio=0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------


//General parameters======================================

static double pi=3.141592;//(rad)
static double g=9.80665;//(m/s^2)

static double rp_limit=0.4;//(rad)
static double y_vel_limit=0.01;//(rad/s)
static double y_d_tangent_deadzone=(double)0.05*y_vel_limit;//(rad/s)
static double T_limit=60;//(N)
//--------------------------------------------------------


//Function declaration====================================

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr &array);
void imu_update(const sensor_msgs::Imu::ConstPtr &imu);
void imu_nav_update(const sensor_msgs::Imu::ConstPtr &imu);
void rel_atti_to_servo_ang(double rel_atti_r, double rel_atti_p);
sensor_msgs::JointState servo_msg_create(double t_r_s, double t_p_s);
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d);
//--------------------------------------------------------


//Control gains===========================================

//integratior(PID) limitation
double integ_limit=2;

//Roll, Pitch PID gains
double Pa=4;
double Ia=0.01;
double Da=0.5;

//Roll, Pitch PID gains (FP fix mode)
double Pa_fp=3;
double Ia_fp=0.01;
double Da_fp=0.5;

//Yaw PID gains
double Py=1;
double Dy=0.01;
//--------------------------------------------------------


//Main====================================================

int main(int argc, char **argv){
	ros::init(argc, argv, "t3_ctrler");
	ros::NodeHandle nh;

	//Publisher
	ros::Publisher PWMs=nh.advertise<std_msgs::Int16MultiArray>("PWMs", 100);
	ros::Publisher servo_angle=nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 100);

	//Subscriber
	ros::Subscriber RC_readings=nh.subscribe("/RC_readings", 100, &arrayCallback);
	ros::Subscriber imu_attitude=nh.subscribe("imu/imu/data", 100, &imu_update);
	ros::Subscriber imu_nav_attitude=nh.subscribe("nav/imu/data", 100, &imu_nav_update);
	ros::Rate loop_rate(200);

	int flag_imu=0;//monitoring imu's availability

	while(ros::ok()){
		if(arr[5]<1300){
			flag_imu=0;

			PWMs_cmd.data.resize(4);
			PWMs_cmd.data[0]=1000;
			PWMs_cmd.data[1]=1000;
			PWMs_cmd.data[2]=1000;
			PWMs_cmd.data[3]=1000;

			rel_atti_to_servo_ang((double)0, (double)0);
		}

		else{
			//Initialize desired yaw
			if(flag_imu!=1){
				y_d=FP_rpy.z;//initial desired yaw setting
				e_r_i=0;//initialize roll integrator
				e_p_i=0;//initialize pitch integrator
	
				if(FP_lin_acc.z<(double)-9)	flag_imu=1;
			}
	
			//TGP ctrl
			r_d=rp_limit*((arr[0]-(double)1500)/(double)500);
			p_d=rp_limit*(-(arr[1]-(double)1500)/(double)500);
			y_d_tangent=y_vel_limit*((arr[2]-(double)1500)/(double)500);
			if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
			y_d+=y_d_tangent;
			T_d=T_limit*((arr[3]-(double)1500)/(double)500);
			//ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", r_d, p_d, y_d, T_d);
			rpyT_ctrl(r_d, p_d, y_d, T_d);	
			
			//FP ctrl, arr[4] controls FP function on/off
			if(arr[4]>=1500)	rel_atti_to_servo_ang(TGP_rpy.x,-TGP_rpy.y);	
			else			rel_atti_to_servo_ang((double)0,(double)0);	
			//else		rel_atti_to_servo_ang(r_d,-p_d);
		}

		//Publish data
		PWMs.publish(PWMs_cmd);
		servo_angle.publish(servo_msg_create(theta_r_s, theta_p_s));
	
		ros::spinOnce();
		loop_rate.sleep();	
	
	}

	return 0;
}
//--------------------------------------------------------


//Functions===============================================

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
	for(int i=0;i<6;i++){
		arr[i]=array->data[i];
	}
	
	return;
}

void imu_update(const sensor_msgs::Imu::ConstPtr &imu){

        geometry_msgs::Quaternion q=imu->orientation;

	//roll
        TGP_rpy.x=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
	
	//pitch        
	double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
        if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
        TGP_rpy.y=asin(temp_y);//pitch 
	
	//yaw
	double temp_z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));
	if(fabs(temp_z-yaw_prev)>pi){
		if(temp_z>=0)		yaw_count-=1;
		else if(temp_z<0)	yaw_count+=1;
	}		
        TGP_rpy.z=temp_z+(double)2*pi*(double)yaw_count;//yaw
	yaw_prev=temp_z;
	//Ref: http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html
	//ROS_INFO("temp_z:%lf, yaw:%lf",temp_z, TGP_rpy.z);
	
	//angular velocity
	TGP_ang_vel=imu->angular_velocity;

	//linear acceleration
        TGP_lin_acc=imu->linear_acceleration;
}

void imu_nav_update(const sensor_msgs::Imu::ConstPtr &imu){

        geometry_msgs::Quaternion q=imu->orientation;

	//roll
        FP_rpy.x=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
	
	//pitch        
	double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
        if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
        FP_rpy.y=asin(temp_y);//pitch 
	
	//yaw
	double temp_z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));
	if(fabs(temp_z-yaw_nav_prev)>pi){
		if(temp_z>=0)		yaw_nav_count-=1;
		else if(temp_z<0)	yaw_nav_count+=1;
	}		
        FP_rpy.z=temp_z+(double)2*pi*(double)yaw_nav_count;//yaw
	yaw_nav_prev=temp_z;
	//Ref: http://marc-b-reynolds.github.io/math/2017/04/18/TaitEuler.html
	//ROS_INFO("temp_z:%lf, yaw:%lf",temp_z, FP_rpy.z);
	
	//angular velocity
	FP_ang_vel=imu->angular_velocity;

	//linear acceleration
        FP_lin_acc=imu->linear_acceleration;
}

void rel_atti_to_servo_ang(double rel_atti_r, double rel_atti_p){

	//Limitation
	if(fabs(rel_atti_r)>=rp_limit)	rel_atti_r=rp_limit*(rel_atti_r/fabs(rel_atti_r));
	if(fabs(rel_atti_p)>=rp_limit)	rel_atti_p=rp_limit*(rel_atti_p/fabs(rel_atti_p));

        //Roll
        double r_r_j=sqrt(pow(r_r_u,2)+pow(r_r_s,2));
        double theta_r_1=atan2(r_r_s,r_r_u);
        double theta_r_2=atan2(r_r_u,r_r_s);
        double k_r=sqrt(pow(r_r_j,2)+pow(r_r_t,2)-2*r_r_j*r_r_t*cos(theta_r_2+rel_atti_r));
        double temp_beta_r=((-pow(r_r_t,2)+pow(r_r_j,2)+pow(k_r,2))/(2*r_r_j*k_r));
        if(fabs(temp_beta_r)>0.9999)    temp_beta_r=(temp_beta_r/fabs(temp_beta_r))*0.9999;
        double beta_r=acos(temp_beta_r);
        double temp_alpha_r=((-pow(k_r,2)+pow(r_r_h,2)+pow(r_r_r,2))/(2*r_r_h*r_r_r));
        if(fabs(temp_alpha_r)>0.9999)   temp_alpha_r=(temp_alpha_r/fabs(temp_alpha_r))*0.9999;
        double alpha_r=acos(temp_alpha_r);
        theta_r_s=asin(r_r_r*sin(alpha_r)/k_r)-theta_r_2-beta_r;

        //Pitch
        double r_p_j=sqrt(pow(r_p_u,2)+pow(r_p_s,2));
        double theta_p_1=atan2(r_p_s,r_p_u);
        double theta_p_2=atan2(r_p_u,r_p_s);
        double k_p=sqrt(pow(r_p_t,2)+pow(r_p_j,2)-2*r_p_t*r_p_j*cos(pi/2-theta_p_1-rel_atti_p));
        double temp_alpha_p=((pow(r_p_j,2)+pow(k_p,2)-pow(r_p_t,2))/(2*r_p_j*k_p));
        if(fabs(temp_alpha_p)>0.9999)   temp_alpha_p=(temp_alpha_p/fabs(temp_alpha_p))*0.9999;
        double alpha_p=acos(temp_alpha_p);
        double beta_p=pi-theta_p_2-alpha_p;
        double temp_theta_p_s=(pow(k_p,2)+pow(r_p_h,2)-pow(r_p_r,2))/(2*k_p*r_p_h);
        if(fabs(temp_theta_p_s)>0.9999) temp_theta_p_s=(temp_theta_p_s/fabs(temp_theta_p_s))*0.9999;
        theta_p_s=acos(temp_theta_p_s)-beta_p;

}

sensor_msgs::JointState servo_msg_create(double t_r_s, double t_p_s){

	sensor_msgs::JointState servo_msg;

	servo_msg.name.resize(2);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";

	servo_msg.position.resize(2);
	servo_msg.position[0]=t_r_s;
	servo_msg.position[1]=t_p_s;

	return servo_msg;
}

void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d){

	double e_r=roll_d-TGP_rpy.x;
	double e_p=pitch_d-TGP_rpy.y;
	double e_y=yaw_d-FP_rpy.z;

	e_r_i+=e_r*((double)1/freq);
	if(fabs(e_r_i)>integ_limit)	e_r_i=(e_r_i/fabs(e_r_i))*integ_limit;
	e_p_i+=e_p*((double)1/freq);
	if(fabs(e_p_i)>integ_limit)	e_p_i=(e_p_i/fabs(e_p_i))*integ_limit;

	tau_r_d=0;
	tau_p_d=0;

	//PID-fp_ctrl_off
	if(arr[4]<1500){
		tau_r_d=Pa*e_r+Ia*e_r_i+Da*(-TGP_ang_vel.x)+(double)0.3;
		tau_p_d=Pa*e_p+Ia*e_p_i+Da*(-TGP_ang_vel.y)+(double)0.2;
	}
	else if(arr[4]>=1500){
		tau_r_d=Pa_fp*e_r+Ia_fp*e_r_i+Da_fp*(-TGP_ang_vel.x)+(double)0.3;
		tau_p_d=Pa_fp*e_p+Ia_fp*e_p_i+Da_fp*(-TGP_ang_vel.y)+(double)0.2;;
	}	
	double tau_y_d=Py*e_y+Dy*(-FP_ang_vel.z);	
	

	//ROS_INFO("xvel:%lf, yvel:%lf, zvel:%lf", TGP_ang_vel.x, TGP_ang_vel.y, TGP_ang_vel.z);
	//ROS_INFO("tr:%lf, tp:%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
}

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des){

	F1=+((double)0.5/l_arm)*tau_p_des-((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F2=+((double)0.5/l_arm)*tau_r_des+((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F3=-((double)0.5/l_arm)*tau_p_des-((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	F4=-((double)0.5/l_arm)*tau_r_des+((double)0.25/b_over_k_ratio)*tau_y_des+(double)0.25*Thrust_des;
	
	//ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F1, F2, F3, F4);
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0]=Force_to_PWM(F1);
	PWMs_cmd.data[1]=Force_to_PWM(F2);
	PWMs_cmd.data[2]=Force_to_PWM(F3);
	PWMs_cmd.data[3]=Force_to_PWM(F4);	
	//ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

double Force_to_PWM(double F){

	double param1=1111.07275742670;
	double param2=44543.2632092715;
	double param3=6112.46876873481;
	//Force=0.0000224500839846839*PWM^2-0.049887353434648*PWM+27.577014233466
	//PWM=1111.07275742670+(44543.2632092715*Force+6112.46876873481)^0.5

	double pwm=param1+sqrt(param2*F+param3);
	if(pwm>1880)	pwm=1880;
	
	return pwm;
}
//--------------------------------------------------------
