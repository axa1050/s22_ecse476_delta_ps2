#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message
#include <double_vec_srv/DblVecSrv.h>
#include <cmath>

bool g_left_lidar_alarm=false; // global var for left lidar alarm
bool g_front_lidar_alarm=false; // global var for front lidar alarm

void leftAlarmCallback(const std_msgs::Bool& alarm_msg)
{
    g_left_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    if (g_left_lidar_alarm) {
        ROS_INFO("left LIDAR alarm received!");
    }
}

void frontAlarmCallback(const std_msgs::Bool& alarm_msg)
{
    g_front_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
    if (g_front_lidar_alarm) {
        ROS_INFO("front LIDAR alarm received!");
    }
}

//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "commander");
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ros::Subscriber left_alarm_subscriber = n.subscribe("left_lidar_alarm",1,leftAlarmCallback);
    ros::Subscriber front_alarm_subscriber = n.subscribe("front_lidar_alarm",1,frontAlarmCallback);
    ros::ServiceClient client = n.serviceClient<double_vec_srv::DblVecSrv>("stdr_rotation_service");
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 0.5; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds


    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
        twist_commander.publish(twist_cmd);
        ros::spinOnce();
        loop_timer.sleep();
    }

    //turn CCW 90 degrees to face north
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    double_vec_srv::DblVecSrv srv;
    double desired_rotation = M_PI/2;
    srv.request.vec_of_doubles.resize(1);
    srv.request.vec_of_doubles[0]=desired_rotation;
    client.call(srv);

    while(ros::ok()) { // do forever
        // move forward until obstacle in front or no wall on left
        twist_cmd.angular.z=0.0; // do not spin 
        twist_cmd.linear.x=speed; //command to move forward
        ros::spinOnce();
        loop_timer.sleep();
        while(!g_front_lidar_alarm && g_left_lidar_alarm) {
            twist_commander.publish(twist_cmd);
            timer+=sample_dt;
            ros::spinOnce();
            loop_timer.sleep();
        }
        //halt if obstacle in front; turn CW 90 degrees
        if(g_front_lidar_alarm && g_left_lidar_alarm) {
            twist_cmd.linear.x = 0.0; //stop moving forward
            timer = 0.0; //reset the timer
            for (int i = 0; i < 10; i++) {
                twist_commander.publish(twist_cmd);
                ros::spinOnce();
                loop_timer.sleep();
            }
            desired_rotation = -M_PI / 2;
            srv.request.vec_of_doubles.resize(1);
            srv.request.vec_of_doubles[0] = desired_rotation;
            ROS_INFO("Turning clockwise");
            client.call(srv);
        }else if(!g_left_lidar_alarm){
            //halt if wall not on left; turn CCW and move forward until wall appears on left again
            twist_cmd.linear.x = 0.0; //stop moving forward
            timer = 0.0; //reset the timer
            for (int i = 0; i < 10; i++) {
                twist_commander.publish(twist_cmd);
                ros::spinOnce();
                loop_timer.sleep();
            }
            desired_rotation = M_PI / 2;
            srv.request.vec_of_doubles.resize(1);
            srv.request.vec_of_doubles[0] = desired_rotation;
            ROS_INFO("Turning counterclockwise");
            client.call(srv);
            twist_cmd.angular.z = 0.0; // do not spin
            twist_cmd.linear.x = speed; //command to move forward
            while (!g_left_lidar_alarm && !g_front_lidar_alarm) {
                twist_commander.publish(twist_cmd);
                timer += sample_dt;
                ros::spinOnce();
                loop_timer.sleep();
            }
        }
    }
    //done commanding the robot; node runs to completion
}

