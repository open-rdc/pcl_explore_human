#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PointStamped.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>

class SearchArea{
    public:
        SearchArea(){
            ros::NodeHandle pnh_("~");
            pnh_.param<std::string>("robot_frame",robot_frame_,"base_link");
            pnh_.param<std::string>("map_frame",map_frame_,"map");
            pub_= nh_.advertise<std_msgs::Bool>("area_flag",100);
            sub_= nh_.subscribe<nav_msgs::OccupancyGrid>("map",100,&SearchArea::mapcallback,this);

            tf_listener_ = new tf::TransformListener();
        }
        void publish();
        void run();

    private:
        void mapcallback(const nav_msgs::OccupancyGridConstPtr &map);
        int coord_to_cell(const double x,const double y);
        bool check_searcharea(const int cell);
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;

        tf::TransformListener *tf_listener_;

        nav_msgs::OccupancyGrid map_;

        std::string robot_frame_;
        std::string map_frame_;
        std::string odom_topic_;

};

int
SearchArea::coord_to_cell(const double x,double y){
    int cell_x=(map_.info.width/2)+std::round(x/map_.info.resolution), 
        cell_y=(map_.info.height/2)+std::round(y/map_.info.resolution);
    return map_.info.width*(cell_y-1)+cell_x+1;
}

bool
SearchArea::check_searcharea(const int cell){
    bool fg=false;

    if(map_.data[cell]==100){
        ROS_DEBUG("occupancy");
        fg=true;
    }
    else if(map_.data[cell]==0){
        ROS_DEBUG("free");
        fg=false;
    }

    else if(map_.data[cell]==-1){
        ROS_DEBUG("unknown");
        fg=false;
    }
    else {
        ROS_WARN("no defined area");
        fg=false;
    }
    return fg;
}

void
SearchArea::mapcallback(const nav_msgs::OccupancyGridConstPtr &map){
    map_.header=map->header;
    map_.info=map->info;
    map_.data=map->data;
}

void 
SearchArea::publish(){
    std_msgs::Bool fg;
    tf::StampedTransform transform;

     if(map_.data.empty()){
        ROS_ERROR("no recieved map_server data");
        return;
    }

    try{
		tf_listener_->waitForTransform(map_frame_,robot_frame_,ros::Time(0),ros::Duration(100));
        tf_listener_->lookupTransform(map_frame_,robot_frame_,ros::Time(0),transform);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
  
    double x=transform.getOrigin().x();
    double y=transform.getOrigin().y();

    fg.data=check_searcharea(coord_to_cell(x,y));
    ROS_DEBUG("flag_status:%d",fg.data);
    pub_.publish(fg);
}

void 
SearchArea::run(){
  ros::Rate loop_rate(20);
  while (ros::ok())
  { 
    if(!map_.data.empty()){
        publish();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "search_area_node");
  
  SearchArea sa;
  sa.run();
  
}