#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//--------------------- Step 4b ---------------------------
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include "kdl/jacobian.hpp"
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
//---------------------------------------------------------

class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();
        // -------- Step 4c -----------
       // void broadcast_listener();    // Not necessary
        // ----------------------------
    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;

        // ----------- Step 2 -----------------

        Eigen::Vector3d _goal_pos_1, _goal_pos_2, _goal_pos_3, _goal_pos_4;
        Eigen::Vector4d _goal_ori_1, _goal_ori_2, _goal_ori_3, _goal_ori_4;

        // ------------------------------------
        // ----------- Step 3a ----------------

        Eigen::Vector3d _goal_pos_5, _goal_pos_6, _goal_pos_7, _goal_pos_8, _goal_pos_9;
        Eigen::Vector4d _goal_ori_5, _goal_ori_6, _goal_ori_7, _goal_ori_8, _goal_ori_9;

        // ------------------------------------
        // ----------- Step 4b ----------------

        Eigen::Vector3d _goal_pos_aruco;
        Eigen::Vector4d _goal_ori_aruco;

        // ------------------------------------

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


};