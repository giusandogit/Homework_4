#include "../include/tf_nav.h"

//---------------------------------------------------------------- Step 4c --------------------------------------------------------------
#include <tf/transform_broadcaster.h>
//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------- Step 4b --------------------------------------------------------------
#include "math.h"        
#include "eigen_conversions/eigen_kdl.h"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"

#include <stdio.h>
#include <iostream>
#include <sstream>
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "Eigen/Core"
#include <Eigen/Dense>
#include <iostream>

// Global variables   (Same as HW03)
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;
// std::vector<double> jnt_pos(7,0.0), init_jnt_pos(7,0.0), jnt_vel(7,0.0), aruco_pose(7,0.0);
// bool robot_state_available = false, aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)  // Step 4b: Callback function I need to retrieve the marker coordinates IN THE MAP FRAME
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}
//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------- Step 4c --------------------------------------------------------------

void PoseCallback(const geometry_msgs::PoseStamped & msg)   // Step 4c: Callback function to broadcast the ArUco pose
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z) );
    tf::Quaternion q;
    q.setX(msg.pose.orientation.x);
    q.setY(msg.pose.orientation.y);
    q.setZ(msg.pose.orientation.z);
    q.setW(msg.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_frame"));  // "aruco_frame" is the frame I need to check with "tf_echo"
}
/*------------------------- This commented listener is NOT necessary ------------------------
void TF_NAV::broadcast_listener() {    // Step 4c: function to listen the broadcast ArUco pose
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "aruco_frame", ros::Time(0), ros::Duration(60.0) );
            listener.lookupTransform( "map", "aruco_frame", ros::Time(0), transform );
        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        r.sleep();
    }
 
}
*/

//---------------------------------------------------------------------------------------------------------------------------------------

TF_NAV::TF_NAV() {

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 0.0, 0.0, 0.0, 1.0;
    _home_pos << -18.0, 2.0, 0.0;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}

void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // ------------------- Step 2b -------------------------

    //tf::TransformListener lis1, lis2, lis3, lis4;
    tf::StampedTransform tr1, tr2, tr3, tr4;

    // -----------------------------------------------------
    // ------------------- Step 3a -------------------------

    tf::StampedTransform tr5, tr6, tr7, tr8, tr9;

    // -----------------------------------------------------
    // ------------------- Step 4b -------------------------

    tf::StampedTransform traruco;

    // -----------------------------------------------------


    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );

            // -------------------------- Step 2b ---------------------------------------
            /*----- Use of 4 different listeners (UNCOMMENT to restore) ------
            lis1.waitForTransform( "map", "goal12a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            lis1.lookupTransform( "map", "goal12a", ros::Time( 0 ), tr1 );
            
            lis2.waitForTransform( "map", "goal22a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            lis2.lookupTransform( "map", "goal22a", ros::Time( 0 ), tr2 );
            
            lis3.waitForTransform( "map", "goal32a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            lis3.lookupTransform( "map", "goal32a", ros::Time( 0 ), tr3 );

            lis4.waitForTransform( "map", "goal42a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            lis4.lookupTransform( "map", "goal42a", ros::Time( 0 ), tr4 );
            */
           
            listener.waitForTransform( "map", "goal12a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal12a", ros::Time( 0 ), tr1 );
            
            listener.waitForTransform( "map", "goal22a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal22a", ros::Time( 0 ), tr2 );
            
            listener.waitForTransform( "map", "goal32a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal32a", ros::Time( 0 ), tr3 );

            listener.waitForTransform( "map", "goal42a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal42a", ros::Time( 0 ), tr4 );

            // --------------------------------------------------------------------------
            // -------------------------- Step 3a ---------------------------------------
           
            listener.waitForTransform( "map", "goal53a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal53a", ros::Time( 0 ), tr5 );
            
            listener.waitForTransform( "map", "goal63a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal63a", ros::Time( 0 ), tr6 );
            
            listener.waitForTransform( "map", "goal73a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal73a", ros::Time( 0 ), tr7 );

            listener.waitForTransform( "map", "goal83a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal83a", ros::Time( 0 ), tr8 );
            
            listener.waitForTransform( "map", "goal93a", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal93a", ros::Time( 0 ), tr9 );

            // --------------------------------------------------------------------------
            // -------------------------- Step 4b ---------------------------------------
           
            listener.waitForTransform( "map", "goalaruco4b", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goalaruco4b", ros::Time( 0 ), traruco );

            // --------------------------------------------------------------------------
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        // --------------------------------- Step 2 ----------------------------------------------------------------
        // I used 4 different tf::Stamped Transform and I changed the orientation quaterion order from w-x-y-z to x-y-z-w
        _goal_pos_1 << tr1.getOrigin().x(), tr1.getOrigin().y(), tr1.getOrigin().z();
        _goal_ori_1 << tr1.getRotation().x(),  tr1.getRotation().y(), tr1.getRotation().z(), tr1.getRotation().w();

        _goal_pos_2 << tr2.getOrigin().x(), tr2.getOrigin().y(), tr2.getOrigin().z();
        _goal_ori_2 << tr2.getRotation().x(),  tr2.getRotation().y(), tr2.getRotation().z(), tr2.getRotation().w();

        _goal_pos_3 << tr3.getOrigin().x(), tr3.getOrigin().y(), tr3.getOrigin().z();
        _goal_ori_3 << tr3.getRotation().x(),  tr3.getRotation().y(), tr3.getRotation().z(), tr3.getRotation().w();

        _goal_pos_4 << tr4.getOrigin().x(), tr4.getOrigin().y(), tr4.getOrigin().z();
        _goal_ori_4 << tr4.getRotation().x(),  tr4.getRotation().y(), tr4.getRotation().z(), tr4.getRotation().w();

        // ----------------------------------------------------------------------------------------------------------
        // --------------------------------- Step 3a ----------------------------------------------------------------

        _goal_pos_5 << tr5.getOrigin().x(), tr5.getOrigin().y(), tr5.getOrigin().z();
        _goal_ori_5 << tr5.getRotation().x(),  tr5.getRotation().y(), tr5.getRotation().z(), tr5.getRotation().w();

        _goal_pos_6 << tr6.getOrigin().x(), tr6.getOrigin().y(), tr6.getOrigin().z();
        _goal_ori_6 << tr6.getRotation().x(),  tr6.getRotation().y(), tr6.getRotation().z(), tr6.getRotation().w();

        _goal_pos_7 << tr7.getOrigin().x(), tr7.getOrigin().y(), tr7.getOrigin().z();
        _goal_ori_7 << tr7.getRotation().x(),  tr7.getRotation().y(), tr7.getRotation().z(), tr7.getRotation().w();

        _goal_pos_8 << tr8.getOrigin().x(), tr8.getOrigin().y(), tr8.getOrigin().z();
        _goal_ori_8 << tr8.getRotation().x(),  tr8.getRotation().y(), tr8.getRotation().z(), tr8.getRotation().w();

        _goal_pos_9 << tr9.getOrigin().x(), tr9.getOrigin().y(), tr9.getOrigin().z();
        _goal_ori_9 << tr9.getRotation().x(),  tr9.getRotation().y(), tr9.getRotation().z(), tr9.getRotation().w();

        // ----------------------------------------------------------------------------------------------------------
        // --------------------------------- Step 4b ----------------------------------------------------------------

        _goal_pos_aruco << traruco.getOrigin().x(), traruco.getOrigin().y(), traruco.getOrigin().z();
        _goal_ori_aruco << traruco.getRotation().x(),  traruco.getRotation().y(), traruco.getRotation().z(), traruco.getRotation().w();

        // ----------------------------------------------------------------------------------------------------------

        r.sleep();
    }    
}

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;
    move_base_msgs::MoveBaseGoal goal4b;

    while ( ros::ok() )
    {
        /*----- Original std::cout lines (UNCOMMENT to restore) ------------------
        std::cout<<"\nInsert 1 to send goal from TF "<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<std::endl;
        */
        /*----- lines for steps until 3a (UNCOMMENT to restore) ------------------
        std::cout<<"Insert 0 to print goal coordinates "<<std::endl;
        std::cout<<"Insert 1 to send goal 1 "<<std::endl;
        std::cout<<"Insert 2 to send goal 2 "<<std::endl;
        std::cout<<"Insert 3 to send goal 3 "<<std::endl;
        std::cout<<"Insert 4 to send goal 4 "<<std::endl;
        std::cout<<"Insert 5 to send home position goal "<<std::endl;
        */
        /*----- lines for steps until 4b (UNCOMMENT to restore) ------------------
        std::cout<<"Insert 0 to print goal coordinates "<<std::endl;      // Step 3a: new output due to the new goals
        std::cout<<"Insert a number from 1 to 9 to send the relative TF goal "<<std::endl;
        std::cout<<"(The goals' sequence for step 3a is 5-1-6-7-8-3-2-9-4) "<<std::endl;
        std::cout<<"Insert 10 to send home position goal "<<std::endl;
        std::cout<<"Your choice: ";
        std::cin>>cmd;
        */
        std::cout<<"Insert 0 to print goal coordinates "<<std::endl;      // Step 4b: new output due to the new goal near the aruco marker
        std::cout<<"Insert a number from 1 to 9 to send the relative TF goal "<<std::endl;
        std::cout<<"Insert 10 to send home position goal "<<std::endl;
        std::cout<<"Insert 11 to start the vision task with the ArUco marker "<<std::endl;
        std::cout<<"Your choice: ";
        std::cin>>cmd;

/* ------------------------ Original if-else cases --------------------------------
        if ( cmd == 1) {    
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos[0];
            goal.target_pose.pose.position.y = _goal_pos[1];
            goal.target_pose.pose.position.z = _goal_pos[2];

            goal.target_pose.pose.orientation.w = _goal_or[0];
            goal.target_pose.pose.orientation.x = _goal_or[1];
            goal.target_pose.pose.orientation.y = _goal_or[2];
            goal.target_pose.pose.orientation.z = _goal_or[3];

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        }
         else if ( cmd == 2) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }
         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
        */
       
        if ( cmd == 1) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_1[0];
            goal.target_pose.pose.position.y = _goal_pos_1[1];
            goal.target_pose.pose.position.z = _goal_pos_1[2];

            goal.target_pose.pose.orientation.x = _goal_ori_1[0];   // I changed the order
            goal.target_pose.pose.orientation.y = _goal_ori_1[1];   // from w-x-y-z to x-y-z-w
            goal.target_pose.pose.orientation.z = _goal_ori_1[2];   //(I did the same for all the 4 goals)
            goal.target_pose.pose.orientation.w = _goal_ori_1[3];

            ROS_INFO("Sending goal 1");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 1");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 2) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_2[0];
            goal.target_pose.pose.position.y = _goal_pos_2[1];
            goal.target_pose.pose.position.z = _goal_pos_2[2];

            goal.target_pose.pose.orientation.x = _goal_ori_2[0];
            goal.target_pose.pose.orientation.y = _goal_ori_2[1];
            goal.target_pose.pose.orientation.z = _goal_ori_2[2];
            goal.target_pose.pose.orientation.w = _goal_ori_2[3];

            ROS_INFO("Sending goal 2");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 2");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 3) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_3[0];
            goal.target_pose.pose.position.y = _goal_pos_3[1];
            goal.target_pose.pose.position.z = _goal_pos_3[2];

            goal.target_pose.pose.orientation.x = _goal_ori_3[0];
            goal.target_pose.pose.orientation.y = _goal_ori_3[1];
            goal.target_pose.pose.orientation.z = _goal_ori_3[2];
            goal.target_pose.pose.orientation.w = _goal_ori_3[3];

            ROS_INFO("Sending goal 3");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 3");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 4) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_4[0];
            goal.target_pose.pose.position.y = _goal_pos_4[1];
            goal.target_pose.pose.position.z = _goal_pos_4[2];

            goal.target_pose.pose.orientation.x = _goal_ori_4[0];
            goal.target_pose.pose.orientation.y = _goal_ori_4[1];
            goal.target_pose.pose.orientation.z = _goal_ori_4[2];
            goal.target_pose.pose.orientation.w = _goal_ori_4[3];

            ROS_INFO("Sending goal 4");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 4");
            else
                ROS_INFO("The base failed to move for some reason");
        }

        // Step 3a: "else if" cases related to the new goals

        else if ( cmd == 5) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_5[0];
            goal.target_pose.pose.position.y = _goal_pos_5[1];
            goal.target_pose.pose.position.z = _goal_pos_5[2];

            goal.target_pose.pose.orientation.x = _goal_ori_5[0];
            goal.target_pose.pose.orientation.y = _goal_ori_5[1];
            goal.target_pose.pose.orientation.z = _goal_ori_5[2];
            goal.target_pose.pose.orientation.w = _goal_ori_5[3];

            ROS_INFO("Sending goal 5");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 5");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 6) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_6[0];
            goal.target_pose.pose.position.y = _goal_pos_6[1];
            goal.target_pose.pose.position.z = _goal_pos_6[2];

            goal.target_pose.pose.orientation.x = _goal_ori_6[0];
            goal.target_pose.pose.orientation.y = _goal_ori_6[1];
            goal.target_pose.pose.orientation.z = _goal_ori_6[2];
            goal.target_pose.pose.orientation.w = _goal_ori_6[3];

            ROS_INFO("Sending goal 6");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 6");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 7) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_7[0];
            goal.target_pose.pose.position.y = _goal_pos_7[1];
            goal.target_pose.pose.position.z = _goal_pos_7[2];

            goal.target_pose.pose.orientation.x = _goal_ori_7[0];
            goal.target_pose.pose.orientation.y = _goal_ori_7[1];
            goal.target_pose.pose.orientation.z = _goal_ori_7[2];
            goal.target_pose.pose.orientation.w = _goal_ori_7[3];

            ROS_INFO("Sending goal 7");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 7");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 8) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_8[0];
            goal.target_pose.pose.position.y = _goal_pos_8[1];
            goal.target_pose.pose.position.z = _goal_pos_8[2];

            goal.target_pose.pose.orientation.x = _goal_ori_8[0];
            goal.target_pose.pose.orientation.y = _goal_ori_8[1];
            goal.target_pose.pose.orientation.z = _goal_ori_8[2];
            goal.target_pose.pose.orientation.w = _goal_ori_8[3];

            ROS_INFO("Sending goal 8");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 8");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 9) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_9[0];
            goal.target_pose.pose.position.y = _goal_pos_9[1];
            goal.target_pose.pose.position.z = _goal_pos_9[2];

            goal.target_pose.pose.orientation.x = _goal_ori_9[0];
            goal.target_pose.pose.orientation.y = _goal_ori_9[1];
            goal.target_pose.pose.orientation.z = _goal_ori_9[2];
            goal.target_pose.pose.orientation.w = _goal_ori_9[3];

            ROS_INFO("Sending goal 9");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has REACHED GOAL 9");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if ( cmd == 10) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }

        // Step 4b: "else if" case for the vision task
        
        else if ( cmd == 11) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _goal_pos_aruco[0];
            goal.target_pose.pose.position.y = _goal_pos_aruco[1];
            goal.target_pose.pose.position.z = _goal_pos_aruco[2];

            goal.target_pose.pose.orientation.x = _goal_ori_aruco[0];
            goal.target_pose.pose.orientation.y = _goal_ori_aruco[1];
            goal.target_pose.pose.orientation.z = _goal_ori_aruco[2];
            goal.target_pose.pose.orientation.w = _goal_ori_aruco[3];

            ROS_INFO("Sending the robot in proximity of the ArUco");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has GOT IN PROXIMITY OF THE MARKER");
            else
                ROS_INFO("The base failed to move for some reason");

            
            std::cout<<"\nRetrieved aruco position\n";    // Debug printing of the retrieved marker pose
            std::cout<<"X: "<<aruco_pose[0]<<std::endl;
            std::cout<<"Y: "<<aruco_pose[1]<<std::endl;
            
            if(true)
            {
                MoveBaseClient ac("move_base", true);
                while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
                }
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
            
                goal.target_pose.pose.position.x = aruco_pose[0] + 1;  // x = x_m + 1
                goal.target_pose.pose.position.y = aruco_pose[1];      // y = y_m
                goal.target_pose.pose.position.z = _goal_pos_aruco[2];

                goal.target_pose.pose.orientation.x = _goal_ori_aruco[0];
                goal.target_pose.pose.orientation.y = _goal_ori_aruco[1];
                goal.target_pose.pose.orientation.z = _goal_ori_aruco[2];
                goal.target_pose.pose.orientation.w = _goal_ori_aruco[3];

            ROS_INFO("Sending the robot in x = x_m + 1, y = y_m");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot has COMPLETED THE VISION TASK");
            else
                ROS_INFO("The base failed to move for some reason");

            }
            std::cout<<"\nFinal robot position\n"<<std::endl;   // Debug printing of the final robot pose
            std::cout<<"X: "<<_cur_pos[0]<<std::endl;
            std::cout<<"Y: "<<_cur_pos[1]<<std::endl;

/*----------------- This code does not work -----------------------------------        
//             KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
//             KDL::Frame base_T_cam(KDL::Rotation::Quaternion(base_pose[3], base_pose[4], base_pose[5], base_pose[6]), KDL::Vector(base_pose[0], base_pose[1], base_pose[2]));

// //---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//             //KDL::Frame base_T_cam(KDL::Rotation::Quaternion(_cur_or[0], _cur_or[1], _cur_or[2], _cur_or[3]), KDL::Vector(_cur_pos[0], _cur_pos[1], _cur_pos[2]));
//             //KDL::Frame base_T_cam(KDL::Rotation::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), KDL::Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));

//             // KDL::Frame base_T_cam; 
//             // base_T_cam.M = KDL::Rotation (_cur_or);                       
//             // base_T_cam.p = KDL::Vector (_cur_pos);  
                                                               

//             //KDL::Rotation X_correction = KDL::Rotation::RotX(0);      // Rotation of Pi around the X axis that I will use to align the frame on the marker to the frame on the camera
//             //KDL::Vector offset(1.0, 0.0, 0.0);                        // Offset of 1 along X in order to
//             //KDL::Frame marker_correction(X_correction, offset);       // Frame defined to apply in a single shot both the desired offset and the orientation correction to the "marker" frame 
//             //marker = marker*marker_correction;                        // Application of the correction to the "marker" frame
            
//             KDL::Frame base_T_object = base_T_cam*cam_T_object; 

//             while(!ac.waitForServer(ros::Duration(5.0))){
//             ROS_INFO("Waiting for the move_base action server to come up");
//             }
//             goal.target_pose.header.frame_id = "map";
//             goal.target_pose.header.stamp = ros::Time::now();
            
//             goal.target_pose.pose.position.x = base_T_object.p[0] + 1;  // x = x_m + 1
//             goal.target_pose.pose.position.y = base_T_object.p[1];      // y = y_m
//             goal.target_pose.pose.position.z = _goal_pos_aruco[2];

//             goal.target_pose.pose.orientation.x = _goal_ori_aruco[0];
//             goal.target_pose.pose.orientation.y = _goal_ori_aruco[1];
//             goal.target_pose.pose.orientation.z = _goal_ori_aruco[2];
//             goal.target_pose.pose.orientation.w = _goal_ori_aruco[3];

//             ROS_INFO("Sending the robot in x = x_m + 1, y = y_m");
//             ac.sendGoal(goal);

//             ac.waitForResult();

//             if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//             ROS_INFO("The mobile robot has REACHED the goal");
//             else
//                 ROS_INFO("The base failed to move for some reason");
*/
        }
        else if ( cmd == 0) {

            //----------------------- Step 2b: debug printing of goals coordinates ---------------------------------

            std::cout<<"\ngoal 1 position (x,y,z)\n"<<_goal_pos_1<<std::endl<<"\ngoal 1 orientation (x,y,z,w)\n"<<_goal_ori_1<<std::endl;
            std::cout<<"\ngoal 2 position (x,y,z)\n"<<_goal_pos_2<<std::endl<<"\ngoal 2 orientation (x,y,z,w)\n"<<_goal_ori_2<<std::endl;
            std::cout<<"\ngoal 3 position (x,y,z)\n"<<_goal_pos_3<<std::endl<<"\ngoal 3 orientation (x,y,z,w)\n"<<_goal_ori_3<<std::endl;
            std::cout<<"\ngoal 4 position (x,y,z)\n"<<_goal_pos_4<<std::endl<<"\ngoal 4 orientation (x,y,z,w)\n"<<_goal_ori_4<<std::endl;

            // Step 3a: coordinates of the new goals
            std::cout<<"\ngoal 5 position (x,y,z)\n"<<_goal_pos_5<<std::endl<<"\ngoal 5 orientation (x,y,z,w)\n"<<_goal_ori_5<<std::endl;
            std::cout<<"\ngoal 6 position (x,y,z)\n"<<_goal_pos_6<<std::endl<<"\ngoal 6 orientation (x,y,z,w)\n"<<_goal_ori_6<<std::endl;
            std::cout<<"\ngoal 7 position (x,y,z)\n"<<_goal_pos_7<<std::endl<<"\ngoal 7 orientation (x,y,z,w)\n"<<_goal_ori_7<<std::endl;
            std::cout<<"\ngoal 8 position (x,y,z)\n"<<_goal_pos_8<<std::endl<<"\ngoal 8 orientation (x,y,z,w)\n"<<_goal_ori_8<<std::endl;
            std::cout<<"\ngoal 9 position (x,y,z)\n"<<_goal_pos_9<<std::endl<<"\ngoal 9 orientation (x,y,z,w)\n"<<_goal_ori_9<<std::endl;

            // Step 4b: coordinates of the pose near the marker
            std::cout<<"\ngoal aruco position (x,y,z)\n"<<_goal_pos_aruco<<std::endl<<"\ngoal aruco orientation (x,y,z,w)\n"<<_goal_ori_aruco<<std::endl;

            //-----------------------------------------------------------------------------------------------------

        }

         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
       
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );

    //-------------- Step 4c ------------------
   // boost::thread broadcast_listener_t( &TF_NAV::broadcast_listener, this );   // Not necessary
    //-----------------------------------------
    
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");

    //----------------------------------- Step 4b ----------------------------------------------
    ros::NodeHandle n;
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    //------------------------------------------------------------------------------------------
    //----------------------------------- Step 4c ----------------------------------------------
    ros::Subscriber pose_sub =  n.subscribe("/aruco_single/pose", 1, PoseCallback);
    //------------------------------------------------------------------------------------------
    
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}