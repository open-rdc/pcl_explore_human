#include<ros/ros.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>

class Target_pose_array{
    public:
        Target_pose_array(){
            sub_=nh_.subscribe<geometry_msgs::PointStamped>("target_point",1,&Target_pose_array::callback,this);
            pub_=nh_.advertise<geometry_msgs::PoseArray>("target_array",1);
        }

        geometry_msgs::PoseArray pose_array_;

    private:
        void callback(geometry_msgs::PointStamped target_point);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        int array_count_=0;
};

void
Target_pose_array::callback(const geometry_msgs::PointStamped target_point)
{
    geometry_msgs::Pose pose;

    pose_array_.header=target_point.header;

    pose.orientation.x=0;
    pose.orientation.y=0;
    pose.orientation.z=0;
    pose.orientation.w=1;

    pose.position.x=target_point.point.x;
    pose.position.y=target_point.point.y;
    pose.position.z=target_point.point.z;

    pose_array_.poses.push_back(pose);
    array_count_++;
    std::cout<<array_count_<<std::endl;
    if(array_count_==100){
        pub_.publish(pose_array_);
        pose_array_.poses.clear();
        array_count_=0;
    }
}

int main(int argc, char **argv)
{
     ros::init (argc, argv, "target_pose_array");

     Target_pose_array tg;

     ros::spin();

     return 0;
}