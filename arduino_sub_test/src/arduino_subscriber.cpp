#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

int Arr[5];
void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array);  

int main(int argc, char **argv){
	ros::init(argc, argv, "arduino_sub_test");
	ros::NodeHandle nh;
	
	ros::Publisher PWMs=nh.advertise<std_msgs::Int16MultiArray>("PWMs", 100);
	ros::Subscriber arduino_sub=nh.subscribe("/RC_readings", 100, &arrayCallback);
	ros::Rate loop_rate(200);
	std_msgs::Int16MultiArray PWM_temp;

	while(ros::ok()){
		PWM_temp.data.resize(4);
		PWM_temp.data[0]=Arr[3];
		PWM_temp.data[1]=Arr[3];
		PWM_temp.data[2]=Arr[3];
		PWM_temp.data[3]=Arr[3];
			
		PWMs.publish(PWM_temp);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
	for(int i=0;i<5;i++){
		Arr[i]=array->data[i];
	}

//	for(int j=0; j<5; j++){
		printf("%d ", Arr[3]);
//	}
	printf("\r");

	return;
}

