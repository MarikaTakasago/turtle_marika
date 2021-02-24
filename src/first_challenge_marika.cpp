#include "first_challenge_marika.h"

FirstChallengeMarika::FirstChallengeMarika():private_nh("")
{
    private_nh.param("hz",hz_,{10});
    private_nh.param("distance",distance_,{1});
    sub_pose = nh.subscribe ("/roomba/odometry",10,&FirstChallengeMarika::pose_callback,this);
    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
}

void FirstChallengeMarika::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    stop_sign_ = 0;

    old_pose = current_pose;
    current_pose = *msg;

    dx_ = old_pose.pose.pose.position.x - current_pose.pose.pose.position.x;
    sum_x_ += dx_;
    if(sum_x_ >= 1 || current_pose.pose.pose.position.x > 1)
    {
        stop_sign_ = 1;
    }

    if(current_pose.pose.pose.orientation.z < 0)
    {
        current_pose.pose.pose.orientation.z += 2*M_PI;
    }

    dtheta_ = old_pose.pose.pose.orientation.z - current_pose.pose.pose.orientation.z;
    sum_theta_ += dtheta_;

    if(sum_theta_ >= 2*M_PI)
    {
        stop_sign_ = 2;
        sum_x_ = 0.0;
    }
}

void FirstChallengeMarika::go_straight()
{
    roomba_500driver_meiji::RoombaCtrl cntl;

    cntl.mode = 11;

    if(stop_sign_  == 1)
    {
        std::cout<<"turn"<<std::endl;
        cntl.cntl.linear.x = 0.0;
        cntl.cntl.angular.z = 0.1;
    }

    else if(stop_sign_ == 0)
    {
        std::cout<<"go!"<<std::endl;
        cntl.cntl.linear.x = 0.5;
    }


    pub_cmd_vel.publish(cntl);

}

void FirstChallengeMarika::process()
{
    ros::Rate loop_rate(hz_);
    sum_x_ = 0;
    sum_theta_ = 0;

    while(ros::ok())
    {
        go_straight();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"first_challenge_marika");
    FirstChallengeMarika first_challenge_marika;
    first_challenge_marika.process();
    return 0;
}
