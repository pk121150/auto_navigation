#include "ros/ros.h"
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h" 

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Joy.h>

#include <sound_play/sound_play.h>

#include "tf/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "map_msgs/OccupancyGridUpdate.h"

#include "move_base_msgs/MoveBaseActionGoal.h"

#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"

#include "std_msgs/String.h"
#include <iostream>
#include <fstream> 
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <cstdlib>

#define GREEN   "\033[32m"      /* Green */
#define WHITE   "\033[37m"      /* White */
#define RED     "\033[31m"      /* Red */
#define YELLOW  "\033[33m"      /* Yellow */
#define CYAN    "\033[36m"      /* Cyan */
using namespace std;

class Way_Points_Navigation
{
public:
    Way_Points_Navigation()
    {
        n.param<bool>("/way_points_navigation_node/soundOpen", soundOpen,false);
        n.param<string>("/way_points_navigation_node/map_frame_id", map_frame_id,"map");
        n.param<string>("/way_points_navigation_node/robot_frame_id", robot_frame_id,"base_footprint");
        n.param<double>("/way_points_navigation_node/stopWaitingTime", stopWaitingTime,2.0);
        n.param<int>("/way_points_navigation_node/goalsNum", goalsNum,1);
        n.param<bool>("/way_points_navigation_node/assign_new_navigation_points", assign_new_navigation_points,false);
        n.param<bool>("/way_points_navigation_node/startWithJoy", startWithJoy, true);
        n.param<double>("/way_points_navigation_node/countDownTime", countDownTime, 3.0);
        n.param<int>("/way_points_navigation_node/loop_count", loop_count,1);
        n.param<double>("/way_points_navigation_node/yaw_goal_tolerance", yaw_goal_tolerance,0.3);
        n.param<double>("/way_points_navigation_node/xy_goal_tolerance", xy_goal_tolerance,0.15);
        
        

        ROS_INFO("Parameters :");
        ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        if( soundOpen)
            ROS_INFO("soundOpen : true");
        else{
            ROS_INFO("soundOpen : false");
        }
        ROS_INFO("map_frame_id : %s",map_frame_id.c_str());
        ROS_INFO("robot_frame_id, : %s",robot_frame_id.c_str());
        ROS_INFO("stopWaitingTime : %lf",stopWaitingTime);
        ROS_INFO("loop_count : %d",loop_count);
        if( assign_new_navigation_points){
            ROS_INFO("assign_new_navigation_points : true");
            ROS_INFO("goalsNum : %d",goalsNum);
        }
        else
            ROS_INFO("assign_new_navigation_points : false");
        if( startWithJoy)
            ROS_INFO("startWithJoy : true");
        else{
            ROS_INFO("startWithJoy : false");
            ROS_INFO("countDownTime : %lf",countDownTime);
        }
        ROS_INFO("xy_goal_tolerance : %lf",xy_goal_tolerance);
        ROS_INFO("yaw_goal_tolerance : %lf",yaw_goal_tolerance);
        ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

        goalPub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        goalArrayPub = n.advertise<geometry_msgs::PoseArray>("/goalArray", 10);
        goalNowPub = n.advertise<geometry_msgs::PoseStamped>("/goalNow", 10);
        cancelPub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
        clickSub = n.subscribe("/clicked_point", 10, &Way_Points_Navigation::clickCallback,this);
        joySub = n.subscribe("/joy", 1, &Way_Points_Navigation::joyCallback,this);

        pathClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

        string package_path = ros::package::getPath("auto_navigation");
        path = package_path + "/path/way_points.dat";
        sound_path = package_path + "/sound";

        // cout << path.c_str()<<endl;
        
        // if not assign new points then read points from file
        if(!assign_new_navigation_points)
        {
            ifstream fin(path.c_str());  
            if (fin.is_open() == false)
            {
                ROS_ERROR("CAN NOT OPEN WAY POINT FILE !!");
                return;
            }else{
                ROS_INFO("OPEN WAY POINT FILE !!");
            }
            string line;
            fin >> line;

            goalsNum = atoi(line.c_str());
            ROS_INFO("goalsNum : %d",goalsNum);

            for(int i=0;i<goalsNum;i++)
            {
                double x,y,qz,qw;
                geometry_msgs::Pose goal;

                fin >> line;
                x = atof(line.c_str());
                fin >> line;
                y = atof(line.c_str());
                fin >> line;
                qz = atof(line.c_str());
                fin >> line;
                qw = atof(line.c_str());

                goal.position.x = x;
                goal.position.y = y;
                goal.orientation.z = qz;
                goal.orientation.w = qw;
                goalList.push_back(goal);
                ROS_INFO("%d goal: (%.2lf %.2lf %.2lf) (%.2lf %.2lf %.2lf %.2f)",i+1
                                                                                ,x
                                                                                ,y
                                                                                ,0.0
                                                                                ,0.0
                                                                                ,0.0
                                                                                ,qz
                                                                                ,qw);
            }
            ROS_INFO("Success to get all the goals !!");

            fin.close();  
            getNavigationPoint = true;
            nowGoal = 0;

            if(!startWithJoy){
                countDown();
                startNavigation = true;
                showGoMsg();
                sendStartSound();
            }else{
                startNavigation = false;
                ROS_INFO("WAIT FOR START COMMAND !! \n\n");
            }
        }else{
            getNavigationPoint = false;
            startNavigation = false;
        }

        arriveGoal = true;
        firstGoal = true;
        stop = false;
        sendStopCommand = false;
        cancelGoal = false;
        nowLoopCount = 0;
        ros::Rate loop_rate(10); 

        // main process
        while (ros::ok()){
            //show goals on rviz
            if(getNavigationPoint)
                pubGoals();
            
            //go navigation
            if(getNavigationPoint && startNavigation){
                if(firstGoal ){
                    firstGoal = false;
                    ROS_INFO("   GO TO FIRST GOAL !!");
                    sendNextWayPoint();
                }else{
                    arriveGoal = checkArrive(goalList[nowGoal]);

                    if(arriveGoal || cancelGoal)
                    {
                        if(arriveGoal)
                            ROS_INFO("   ARRIVE GOAL %d !!",nowGoal+1); 
                        nowGoal++;
                        
                        if(nowGoal == goalsNum)
                        {
                            nowGoal = 0;
                            nowLoopCount++;
                            if(nowLoopCount == loop_count)
                            {
                                if(cancelGoal){
                                    sendcancelCommand();
                                    cancelGoal = false;
                                }
                                sendFinishAllTasksSound();
                                cout << GREEN << "FINISHED ALL MISSIONS, WAIT FOR COMMAND !!" << WHITE <<endl<<endl;
                                startNavigation = false;
                                nowLoopCount = 0;
                                firstGoal = true;
                                stop = false;
                                sendStopCommand = false;
                                continue;
                            }
                        }
                        if(arriveGoal){
                            sendPresentTargetArrivedSound();
                            ros::Duration(stopWaitingTime).sleep();
                        }
                        if(cancelGoal)
                            sendcancelCommand();
                    
                        if(!stop){
                            sendGoToNextTargetdSound();
                            sendNextWayPoint();
                            ROS_INFO("   GO TO NEXT GOAL !!");
                        }
                        cancelGoal = false;
                    }else{
                        // check whether send stop command
                        if(stop){
                            if(!sendStopCommand){
                                sendcancelCommand();
                                sendStopCommand = true;
                            }
                        }else{
                            if(sendStopCommand){
                                sendNextWayPoint();
                                sendStopCommand = false;
                            }
                        }
                    }


                    
                }
            }else{//wait for points assign and start navigation command
                ;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
        cout << GREEN;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        cout << "!!!!!! STOP WAY POINTS NAVIGATION !!!!!!" << endl;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl << endl << endl;
        cout << WHITE;

        if(!assign_new_navigation_points)
            return;
        
        if(goalList.size() != goalsNum)
        {
            cout << RED;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "!!!!!! Way points number not correct, no save way points!!!!!!" << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl << endl << endl;
            cout << WHITE;
            return;
        }
        ofstream fout(path.c_str());  
        if(fout.is_open() == false)
        { 
            cout << RED << "CAN NOT OPEN WAY POINT FILE !!" << endl;
            cout << "SAVE WAY POINT TO FILE FAIL!!" << WHITE << endl ;
            return;
        }else{
            //cout << "Open way point file !!" << endl;

            fout << goalsNum << "\n";
            for(int i=0;i<goalsNum;i++)
            {
                fout <<  goalList[i].position.x << "\n";
                fout <<  goalList[i].position.y << "\n";
                fout <<  goalList[i].orientation.z << "\n";
                fout <<  goalList[i].orientation.w << "\n";
            }
            cout << GREEN <<"!!! SUCCESS TO SAVE WAY POINTS!!!" << WHITE << endl << endl;
        }
        
        fout.close();
    }



    //check whether to start navigation
    void joyCallback(const sensor_msgs::Joy::ConstPtr msg)
    {
        // start navigation command
        if(msg->buttons[5] == 1 && !startNavigation && startWithJoy )
        {
            if(getNavigationPoint)
            {
                showGoMsg();
                sendStartSound();
                startNavigation = true;
            }else{
                ROS_WARN("Please set way points first !!");
            }
        }

        // stop command
        if(msg->buttons[2] == 1 && startNavigation)
        {
            if(stop)
            {
                stop = false;
                sendContinueToPerformTaskSound();
                ROS_INFO("   GO TO NEXT GOAL !!");              
            }else{
                stop = true;
                sendPausePerformingTaskSound();     
                ROS_INFO("   STOP PERFORMING TASK FOR A MOMENT !!");
            }
        }

        if(msg->buttons[0] == 1 && startNavigation)
        {
            cancelGoal = true;
            sendCancelTargetSound();
            ROS_INFO("   CANCEL PRESENT GOAL !!");   
        }
        
    }


    //set way point
    void clickCallback(const geometry_msgs::PointStampedConstPtr& msg)
    {
        static int clickNum = 0;

        //start navigation by click one more time
        if(getNavigationPoint  && !startWithJoy && !startNavigation)
        {
            countDown();
            showGoMsg();
            
            ros::Duration(1).sleep();
            startNavigation = true;
            return;
        }

        if(!getNavigationPoint && msg->header.frame_id == map_frame_id )
	    {
            clickNum++;
            geometry_msgs::Pose goal;
            if(clickNum%2 == 1)//position assign
		    {
                goal.position = msg->point;
                goal.orientation.x = 0;
		        goal.orientation.y = -0.7071068;
			    goal.orientation.z = 0;
			    goal.orientation.w = 0.7071068;

                goalList.push_back(goal);
                nowGoal = clickNum/2;
            }else{//orientation assign
                double dx = msg->point.x - goalList.back().position.x;
			    double dy = msg->point.y - goalList.back().position.y;
			    double yaw =  atan2(dy,dx);
                tf::Quaternion q;
			    q.setRPY( 0, 0, yaw);
                goal.position = goalList.back().position;
                goal.orientation.x = q.getX();
		        goal.orientation.y = q.getY();
			    goal.orientation.z = q.getZ();
			    goal.orientation.w = q.getW();

                goalList.pop_back();
                goalList.push_back(goal);

                ROS_INFO("%d GOAL: (%.2lf %.2lf %.2lf) (%.2lf %.2lf %.2lf %.2f)",clickNum/2
                                                                                ,goal.position.x
                                                                                ,goal.position.y
                                                                                ,goal.position.z
                                                                                ,goal.orientation.x
                                                                                ,goal.orientation.y
                                                                                ,goal.orientation.z
                                                                                ,goal.orientation.w);
                if(clickNum == goalsNum*2)
                {
                    getNavigationPoint = true;
                    ROS_INFO("GET ALL WAY POINTS, WAIT FOR START COMMAND !!\n\n");
                    nowGoal = 0;
                }    
            }
            pubGoals();
        }
    }

    //show goals om rviz
    void pubGoals()
    {
        geometry_msgs::PoseArray goalArray;
        geometry_msgs::PoseStamped goalNow;
        goalArray.poses = goalList; 
        goalArray.header.frame_id = "map";
        goalNow.pose = goalList[nowGoal];
        goalNow.header.frame_id = "map";

        goalArrayPub.publish(goalArray);
        goalNowPub.publish(goalNow);
    }

    void showGoMsg()
    {
        cout << GREEN;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        cout << "!!!!!! START WAY POINTS NAVIGATION !!!!!!" << endl;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl << endl << endl;
        cout << WHITE;
    }

    void countDown(){
        cout << CYAN ;
        for(double i = countDownTime;i > 0 ;i--)
        {
            cout <<"!!!   "<<(int)i << "   !!!"<<endl;
            ros::Duration(1).sleep();   
        }
        cout << WHITE ;
    }

    bool checkArrive(geometry_msgs::Pose goal){
        geometry_msgs::Pose nowPose;

        if(!getNowPosition(nowPose)) 
            return false;

        double x_dis = goal.position.x - nowPose.position.x;
        double y_dis = goal.position.y - nowPose.position.y;
        double dis = pow(pow(x_dis,2)+pow(y_dis,2),0.5);

        tf::Quaternion goal_tf(goal.orientation.x,
                               goal.orientation.y,
                               goal.orientation.z,
                               goal.orientation.w);  
        tf::Quaternion now_tf( nowPose.orientation.x,
                               nowPose.orientation.y,
                               nowPose.orientation.z,
                               nowPose.orientation.w); 
        double goal_yaw = tf::getYaw(goal_tf);
        double now_yaw = tf::getYaw(now_tf);
        double dYaw = abs(goal_yaw - now_yaw);
        if(dYaw > M_PI)
            dYaw = M_PI*2 - dYaw;  
        //cout << dis <<" "<< dYaw<<" "<<dYaw/M_PI*180 <<endl;
        if(dis < xy_goal_tolerance && dYaw < yaw_goal_tolerance)
            return true;
        else
            return false;
        
    }

    bool getNowPosition(geometry_msgs::Pose &nowPose){
        
        tf::StampedTransform transform;
        try{
            // tfListener.waitForTransform(map_frame_id, robot_frame_id, ros::Time::now(), ros::Duration(0.1));
            // tfListener.lookupTransform(map_frame_id, robot_frame_id,  ros::Time::now(), transform);        
            tfListener.lookupTransform(map_frame_id, robot_frame_id,  ros::Time(0), transform);                     
        }
        catch (tf::TransformException ex){
            //ROS_WARN("Failed to compute transform from map_frame, to robot_frame, ignore pose assigned (%s)", ex.what());
            return false;
        }

        nowPose.position.x = transform.getOrigin().x();
        nowPose.position.y = transform.getOrigin().y();
        nowPose.position.z = transform.getOrigin().z();
        nowPose.orientation.x = transform.getRotation().x();
        nowPose.orientation.y = transform.getRotation().y();
        nowPose.orientation.z = transform.getRotation().z();
        nowPose.orientation.w = transform.getRotation().w();

        return true;
    }

    void sendNextWayPoint(){
        geometry_msgs::Pose nowPose;
        nav_msgs::Path path;

        bool getNowPose = getNowPosition(nowPose);
        bool getPath = getTargetPath(path,nowPose,goalList[nowGoal]);

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        if(getPath){
            goal.pose = path.poses[0].pose;
            goalPub.publish(goal);

            while(!checkArrive(goal.pose)){
                ros::Duration(0.1).sleep();
            }
        }
        goal.pose = goalList[nowGoal];

        goalPub.publish(goal);

        // geometry_msgs::PoseStamped goal;
        // goal.header.frame_id = "map";
        // goal.pose = goalList[nowGoal];

        // goalPub.publish(goal);
    }

    void sendcancelCommand(){
        actionlib_msgs::GoalID cancel_msg;
        cancelPub.publish(cancel_msg);    
    }

    bool getTargetPath(nav_msgs::Path& path, geometry_msgs::Pose start, geometry_msgs::Pose goal)
    {
        nav_msgs::GetPlan srv;
        srv.request.start.pose = start;
        srv.request.start.header.frame_id = map_frame_id;
        srv.request.goal.pose = goal;
        srv.request.goal.header.frame_id = map_frame_id;
        srv.request.tolerance = 0.0;

        if(pathClient.call(srv))
        {
            path = srv.response.plan;
            return true;
        }
        else
        {
            // ROS_ERROR("Failed to call service /move_base/make_plan");
            return false;
        }
    } 
    

    void sendStartSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/start_to_perform_task.mp3");
    }

    void sendPresentTargetArrivedSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/present_target_is_arrived.mp3");
    }

    void sendGoToNextTargetdSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/go_to_next_target.mp3");
    }

    void sendFinishAllTasksSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/finish_all_tasks.mp3");
    }

    void sendPausePerformingTaskSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/pause_performing_task.mp3");
    }

    void sendContinueToPerformTaskSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/continue_to_perform_task.mp3");
    }

    void sendCancelTargetSound(){
        if(soundOpen)
            sc.playWave(sound_path+"/cancel_target.mp3");
    }

    ros::NodeHandle n;

    //params
    string map_frame_id,robot_frame_id;
    double stopWaitingTime,countDownTime,yaw_goal_tolerance,xy_goal_tolerance;
    int goalsNum,loop_count;
    bool assign_new_navigation_points,startWithJoy,soundOpen;

    std::vector<geometry_msgs::Pose> goalList;

    bool getNavigationPoint;
    bool startNavigation;
    bool arriveGoal;
    bool firstGoal;
    bool stop;
    bool sendStopCommand;
    bool cancelGoal;

    int nowGoal;
    int nowLoopCount;

    ros::Publisher cancelPub;
    ros::Publisher goalPub;
    ros::Publisher goalArrayPub;
    ros::Publisher goalNowPub;
    ros::Subscriber clickSub;
    ros::Subscriber joySub; 
    
    tf::TransformListener tfListener;

    sound_play::SoundClient sc;

    string path;
    string sound_path;

    ros::ServiceClient pathClient;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "way_points_navigation_node");

    Way_Points_Navigation way_points_navigation_node;
    
    return (0);
}