#include "first_challenge_marika.h"

FirstChallengeMarika::FirstChallengeMarika():private_nh("")
{
    private_nh.param("hz",hz_,{10});
    private_nh.param("distance",distance_,{1});
    sub_pose = nh.subscribe ("/roomba/Odometry",10,&FirstChallengeMarika::pose_callback,this);
}

void FirstChallengeMarika::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = *msg;
}

void FirstChallengeMarika::go_straight(int i)
{
    if(i<=10)
    {
       X = current_pose.pose.pose.position.x;
    }

    x = current_pose.pose.pose.position.x;
    std::cout<<x<<std::endl;
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.5;
    if(x - X >= 1){
        cmd_vel.linear.x = 0.0;
    }

    pub_cmd_vel.publish(cmd_vel);

}

void FirstChallengeMarika::process()
{
    ros::Rate loop_rate(hz_);
    int i = 0;

    while(ros::ok())
    {
        go_straight(i);
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"first_challenge_marika");
    FirstChallengeMarika first_challenge_marika;
    first_challenge_marika.process();
    return 0;
}
