#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
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

int status = 1;
float now_x = 0,now_y = 0;
float pre_x ,pre_y,dis,dis_pre;
bool first = true,stuck = false;
bool chooseGoalFinished = false;
bool arriveGoal = false;
bool startNav = false;
int goalNum;
int goalNumChoosed = 0;
double goalX[100],goalY[100],goalW[100];
string map_frame_id;
double stopWaitingTime;
int clickNum = 0;
geometry_msgs::PoseArray goalArray; 

int last_status = 1;
//float array[3][2] = {{14.866, -6.33 }, {32.31, 5.79 }, {4.36,  13.537 }};  //LAB

//float array[3][2] = {{-8.99, 1.93 }, {-7.98, -2.67 }, {-4.66, -2.32 }};  //HOME


void clickCallback(geometry_msgs::PointStamped msg)
{
	if(chooseGoalFinished && startNav == false)
	{
		startNav = true;
		cout<<endl<<"Start navigation !!"<<endl;
	}

	if(clickNum < goalNum*2 && msg.header.frame_id == map_frame_id)
	{
		clickNum++;
		if(clickNum%2 == 1)
		{
			goalX[goalNumChoosed] = msg.point.x;
			goalY[goalNumChoosed] = msg.point.y;
			cout<<"GOAL "<<goalNumChoosed<<" = ("<<goalX[goalNumChoosed]<<" , "<<goalY[goalNumChoosed]<<")"<<endl;

			geometry_msgs::Pose pose;
    		pose.position.x = goalX[goalNumChoosed];
    		pose.position.y = goalY[goalNumChoosed];
    		pose.position.z = 0;
			pose.orientation.x = 0;
		    pose.orientation.y = -0.7071068;
			pose.orientation.z = 0;
			pose.orientation.w = 0.7071068;




    		goalArray.poses.push_back(pose);
		}else{
			double dx = msg.point.x - goalX[goalNumChoosed];
			double dy = msg.point.y - goalY[goalNumChoosed];
			goalW[goalNumChoosed] =  atan2(dy,dx);


			tf2::Quaternion q;
			q.setRPY( 0, 0, goalW[goalNumChoosed]);
				

			geometry_msgs::Pose pose;
    		pose.position.x = goalX[goalNumChoosed];
    		pose.position.y = goalY[goalNumChoosed];
    		pose.position.z = 0;
			pose.orientation.x = q.getX();
		    pose.orientation.y = q.getY();
			pose.orientation.z = q.getZ();
			pose.orientation.w = q.getW();



			goalArray.poses.pop_back();
    		goalArray.poses.push_back(pose);

			

			goalNumChoosed++;
		}
		
		

	}

	if(goalNumChoosed == goalNum && chooseGoalFinished == false){
		chooseGoalFinished = true;
		cout<<"Navigation Goals :"<<endl;
		// for(int i = 0;i<goalNum;i++)
		// {
		// 	cout<<"("<<goalX[i]<<" , "<<goalY[i]<<")"<<endl;
		// }
		cout<<endl<<"click any point to start navigation"<<endl;
	}
}



// void statusCallback(const std_msgs::String::ConstPtr& msg)
// {
// 	if(strcmp (msg->data.c_str(),"idle") == 0){
// 	//if(msg->data.c_str() == "idle")
// 		status = 0;
// 		arriveGoal = true;
// 	}
// 	else
// 		status = 1;

// 	//cout<<status<<endl;
// }


void statusCallback2(actionlib_msgs::GoalStatusArray msg)
{
	int a;
	a = msg.status_list[0].status;
	// for(int i = 0;i<msg.status_list.size();i++){
	// 	int b = msg.status_list[i].status;	
	// 	cout << b <<endl;
	// }
	// cout << endl; 
	if((!first) && status == 1){
        
		

		//ROS_INFO("status:%d  %s ",msg.status_list[0].status,msg.status_list[0].text.c_str());

		if(a == 4)
			stuck = true;
		else
			stuck = false;

		
	}

	if(msg.status_list.size() == 1)
	{
		if(last_status == 1 && a == 3)
		{
			arriveGoal = true;
		}
		last_status = a;
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
	// ros::param::get("/multi_goals/stopDistance",stopDis);
	ros::param::get("/multi_goals/map_frame_id",map_frame_id);
	ros::param::get("/multi_goals/stopWaitingTime",stopWaitingTime);

	cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
	// cout<<"Distance to change goal: "<<stopDis<<endl;
	cout<<"Map frame id: "<<map_frame_id<<endl;
	cout<<"Time to wait when arriving goal: "<<stopWaitingTime<<endl<<endl;
	cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
    cout<<"Please input numbers of goals: ";
	cin>>goalNum;
	
    goalArray.header.frame_id = "map";



	ros::Publisher pub_goal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
	ros::Publisher pub_goalArray = n.advertise<geometry_msgs::PoseArray>("/goalArray", 1000);

	// ros::Subscriber sub  = n.subscribe("/cmd_vel_mux/active", 1000, statusCallback);
    ros::Subscriber sub2 = n.subscribe("/move_base/status", 1000, statusCallback2);
	ros::Subscriber sub_click = n.subscribe("/clicked_point", 1000, clickCallback);

    ros::Rate loop_rate(10);


	ros::Duration(1).sleep();
    cout<<endl<<"clicked navi points on rviz !!"<<endl<<endl;
	while (ros::ok())
    {
		if(chooseGoalFinished && startNav){


			move_base_msgs::MoveBaseActionGoal msg;
	    	tf::StampedTransform transform;

	    	// try{
	    	// 	listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
	    	// }catch(tf::TransformException ex){
	    	// 	ROS_ERROR("%s",ex.what());
	    	// 	ros::Duration(0.1).sleep();
			// }

			// now_x = transform.getOrigin().x();
	    	// now_y = transform.getOrigin().y();

			// pre_x = goalX[count];
			// pre_y = goalY[count];

			// dis = pow(pow(now_x - pre_x,2) + pow(now_y - pre_y,2) , 0.5);

			//cout<<"dis = "<<dis<<endl<<endl;

	    	if((first && status == 0) || stuck == true)
			{
				msg.goal.target_pose.header.frame_id = "map";
				msg.goal.target_pose.pose = goalArray.poses[0];
				// msg.goal.target_pose.pose.position.x = goalX[count];
				// msg.goal.target_pose.pose.position.y = goalY[count];
	      		// msg.goal.target_pose.pose.orientation.z = 0;
				// msg.goal.target_pose.pose.orientation.w = 1;
				pub_goal.publish(msg);
				stuck = false;
				first = false;
				arriveGoal = false;

			}

			// if(dis < stopDis)
			if(arriveGoal)
			{
				
				if(count == goalNum-1)
					count = 0;
				else
					count++;

				cout << "Arrive goal!!"<<endl<<endl; 
				
				ros::Duration(stopWaitingTime).sleep();
				cout << "Going to new goal!!"<<endl<<endl; 
				msg.goal.target_pose.header.frame_id = "map";
				msg.goal.target_pose.pose = goalArray.poses[count];
				// msg.goal.target_pose.pose.position.x = goalX[count];
				// msg.goal.target_pose.pose.position.y = goalY[count];
				// tf2::Quaternion q;
				// q.setRPY( 0, 0, goalW[count]);
				// msg.goal.target_pose.pose.orientation.x = q.getX();
				// msg.goal.target_pose.pose.orientation.y = q.getY();
				// msg.goal.target_pose.pose.orientation.z = q.getZ();
				// msg.goal.target_pose.pose.orientation.w = q.getW();
				pub_goal.publish(msg);

				arriveGoal = false;
			}

		}
		if(clickNum > 0)
			pub_goalArray.publish(goalArray);
		ros::spinOnce();
		loop_rate.sleep();
	}



	return 0;
}
