#include "first_challenge_marika.h"

FirstChallengeMarika::FirstChallengeMarika():private_nh("")
{
    private_nh.param("hz",hz_,{50});
    private_nh.param("distance",distance_,{1});
    private_nh.param("stop_distance_",stop_distance_,{0.5});
    private_nh.param("stop_sign_",stop_sign_,{0});
    sub_pose = nh.subscribe("/roomba/odometry",10,&FirstChallengeMarika::pose_callback,this);
    sub_range = nh.subscribe("/scan",10,&FirstChallengeMarika::range_callback,this);
    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
}

void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw)
{
    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
}

void FirstChallengeMarika::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    old_pose = current_pose;
    current_pose = *msg;
    if(!pose_checker) old_pose = current_pose;
    pose_checker = true;

    dx_ = current_pose.pose.pose.position.x - old_pose.pose.pose.position.x;
    sum_x_ += dx_;

    old_yaw = yaw;
    GetRPY(current_pose.pose.pose.orientation,roll,pitch,yaw);
    if(yaw*old_yaw<0) dtheta_ = 0.0;
    else dtheta_ = fabs(yaw - old_yaw);

    std::cout<<"current_x:"<<current_pose.pose.pose.position.x<<std::endl;
    std::cout<<"current_yaw:"<<yaw<<std::endl;

    if(sum_x_ >= 1 || stop_sign_ == 1)
    {
        stop_sign_ = 1;
        sum_x_ = 0.0;
        sum_theta_ += dtheta_;
        std::cout<<"sum_theta:"<<sum_theta_<<std::endl;
        if(sum_theta_ >= 2*M_PI) stop_sign_ = 2;
    }
}

void FirstChallengeMarika::range_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    current_range = *msg;

    double use_range = current_range.ranges.size()/2;
    // std::cout<<current_range.ranges.size()<<std::endl;
    if(current_range.ranges.size() >= 1000)
    {
        // std::cout<<current_range.ranges[539]<<std::endl;
        if(stop_sign_ = 2)
        {
            std::cout<<"dist:"<<current_range.ranges[use_range]<<std::endl;
            if(current_range.ranges[use_range] <= stop_distance_) stop_sign_ = 3;
        }
    }

}

void FirstChallengeMarika::go_straight()
{
    roomba_500driver_meiji::RoombaCtrl cmd_vel;

    cmd_vel.mode = 11;

    if(stop_sign_ == 0)
    {
        sum_theta_ = 0.0;
        std::cout<<"go!"<<std::endl;
        cmd_vel.cntl.linear.x = 0.2;
        cmd_vel.cntl.angular.z = 0.0;
    }
    else if(stop_sign_  == 1)
    {
        std::cout<<"turn"<<std::endl;
        cmd_vel.cntl.linear.x = 0.0;
        cmd_vel.cntl.angular.z = 0.5;
        // sum_theta_ += dtheta_;
        // std::cout<<"sum_theta:"<<sum_theta_<<std::endl;
        // if(sum_theta_ >= 2*M_PI) stop_sign_ = 2;
    }

    else if(stop_sign_ == 2)
    {
        sum_x_ = 0.0;
        sum_theta_ = 0.0;
        std::cout<<"go-go!"<<std::endl;
        cmd_vel.cntl.angular.z = 0.0;
        cmd_vel.cntl.linear.x = 0.2;
    }

    else if(stop_sign_ == 3)
    {
        sum_x_  = 0.0;
        cmd_vel.cntl.linear.x = 0.0;
        cmd_vel.cntl.angular.z = 0.0;
        std::cout<<"stop..."<<std::endl;
    }


    pub_cmd_vel.publish(cmd_vel);

}

void FirstChallengeMarika::process()
{
    ros::Rate loop_rate(hz_);
    sum_x_ = 0;
    sum_theta_ = 0;
    old_yaw = 0;
    yaw = 0;

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
