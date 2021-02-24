#ifndef FIRST_CHALLENGE_MARIKA_H
#define FIRST_CHALLENGE_MARIKA_H

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"


class FirstChallengeMarika
{
    public:
        FirstChallengeMarika();
        void process();

    private:
        //method
        void pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void go_straight();

        //parameter
        int hz_;
        double distance_;

        int stop_sign_;
        double dx_;
        double dtheta_;
        double sum_x_;
        double sum_theta_;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_pose;
        ros::Publisher pub_cmd_vel;
        nav_msgs::Odometry old_pose;
        nav_msgs::Odometry current_pose;
};
#endif
