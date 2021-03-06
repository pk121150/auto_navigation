#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>

using namespace std;

int status;
float now_x = 0,now_y = 0;
float pre_x ,pre_y,dis,dis_pre;
bool first = true,stuck = false;
bool chooseGoalFinished = false;
bool startNav = false;
int goalNum;
int goalNumChoosed = 0;
double goalX[100],goalY[100];
//float array[3][2] = {{14.866, -6.33 }, {32.31, 5.79 }, {4.36,  13.537 }};  //LAB

//float array[3][2] = {{-8.99, 1.93 }, {-7.98, -2.67 }, {-4.66, -2.32 }};  //HOME


void clickCallback(geometry_msgs::PointStamped msg)
{
	if(chooseGoalFinished && startNav == false)
	{
		startNav = true;
		cout<<endl<<"Start navigation !!"<<endl;
	}

	if(goalNumChoosed < goalNum)
	{
		goalNumChoosed++;
		goalX[goalNumChoosed-1] = msg.point.x;
		goalY[goalNumChoosed-1] = msg.point.y;

	}

	if(goalNumChoosed == goalNum && chooseGoalFinished == false){
		chooseGoalFinished = true;
		cout<<"Navigation Goals :"<<endl;
		for(int i = 0;i<goalNum;i++)
		{
			cout<<"("<<goalX[i]<<" , "<<goalY[i]<<")"<<endl;
		}
		cout<<endl<<"click any point to start navigation"<<endl;
	}
}



void statusCallback(const std_msgs::String::ConstPtr& msg)
{
	if(strcmp (msg->data.c_str(),"idle") == 0)
	//if(msg->data.c_str() == "idle")
		status = 0;
	else
		status = 1;

	//cout<<status<<endl;
}


void statusCallback2(actionlib_msgs::GoalStatusArray msg)
{

	if((!first) && status == 1){
        	int a;
		a = msg.status_list[0].status;

		ROS_INFO("status:%d  %s ",msg.status_list[0].status,msg.status_list[0].text.c_str());

		if(a == 4)
			stuck = true;
		else
			stuck = false;
	}

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "go_rviz");
	ros::NodeHandle n;

  tf::TransformListener listener;

	status = 0;
  int count = 0;

  double stopDis;

  stopDis = 0.5;
	//ros::param::get("numOfGoals", goalNum);
	ros::param::get("stopDistance",stopDis);
	cout<<"Distance to change goal: "<<stopDis<<endl;
  cout<<"Numbers of goals: ";
	cin>>goalNum;
	cout<<endl<<"clicked navi points on rviz !!"<<endl;




	ros::Publisher pub_goal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);

	ros::Subscriber sub  = n.subscribe("/cmd_vel_mux/active", 1000, statusCallback);
  ros::Subscriber sub2 = n.subscribe("/move_base/status", 1000, statusCallback2);
	ros::Subscriber sub_click = n.subscribe("/clicked_point", 1000, clickCallback);

  ros::Rate loop_rate(10);


	ros::Duration(10).sleep();

	while (ros::ok())
  {
		if(chooseGoalFinished && startNav){


			move_base_msgs::MoveBaseActionGoal msg;
	    tf::StampedTransform transform;

	    try{
	    	listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
	    }catch(tf::TransformException ex){
	    	ROS_ERROR("%s",ex.what());
	    	ros::Duration(0.1).sleep();
	  	}

			now_x = transform.getOrigin().x();
	    now_y = transform.getOrigin().y();

			pre_x = goalX[count];
			pre_y = goalY[count];

			dis = pow(pow(now_x - pre_x,2) + pow(now_y - pre_y,2) , 0.5);

			//cout<<"dis = "<<dis<<endl<<endl;

	    if((first && status == 0) || stuck == true)
			{
				msg.goal.target_pose.header.frame_id = "map";
				msg.goal.target_pose.pose.position.x = goalX[count];
				msg.goal.target_pose.pose.position.y = goalY[count];
	      msg.goal.target_pose.pose.orientation.z = 0;
				msg.goal.target_pose.pose.orientation.w = 1;
				pub_goal.publish(msg);
				stuck = false;
				first = false;

			}

			if(dis < stopDis)
			{
				if(count == goalNum-1)
					count = 0;
				else
					count++;

				msg.goal.target_pose.header.frame_id = "map";
				msg.goal.target_pose.pose.position.x = goalX[count];
				msg.goal.target_pose.pose.position.y = goalY[count];
				msg.goal.target_pose.pose.orientation.z = 0;
				msg.goal.target_pose.pose.orientation.w = 1;
				pub_goal.publish(msg);
			}

		}

		ros::spinOnce();
		loop_rate.sleep();
	}



	return 0;
}
