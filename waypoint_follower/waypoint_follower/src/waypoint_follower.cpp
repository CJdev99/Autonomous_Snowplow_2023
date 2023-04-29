#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <fstream>
#include <cmath>
#include <tf/tf.h>
#include <signal.h>
#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


#define QUADRATIC(x,y) (sqrt(pow(x,2)+pow(y,2)))
uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}
void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}
class MainNode
{

    public:
        MainNode();
    public:

        void currentPoseLoop();
        void saveWaypointCallback(const std_msgs::Empty::ConstPtr& msg);
        void toggleNavigationCallback(const std_msgs::Empty::ConstPtr& msg);
        void clickedPointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void resultCallback(const move::PoseWithCovarianceStamped::ConstPtr& msg);
        //void stopNavigationCallback(const std_msgs::Empty::ConstPtr& msg);
        void loadQueue();
        int run();
        void moveBaseClient();
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber save_waypoint_sub_;
        ros::Subscriber start_nav_sub_;
        ros::Subscriber stop_nav_sub_;
        ros::Subscriber clicked_point_sub_;
        ros::Subscriber status_sub;
        ros::Publisher cancel_waypoint;
        tf::TransformListener listener_;
        std::string map_frame_;
        std::string base_link_frame_;
        std::string waypoint_file_path_;
        std::vector<geometry_msgs::PoseStamped> waypoint_queue_;
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped dummy_pose;
        
        
        uint32_t starttime;
        uint32_t hstimer;
        uint32_t mstimer;
        uint32_t lstimer;
        double distanceToGoal;
        std::ofstream csv_file;
        bool navigation_state;
        bool reachedGoal;
        


};

MainNode::MainNode():
    starttime(0),
    hstimer(0),
    mstimer(0),
    lstimer(0),
    distanceToGoal(9999),
    navigation_state(false),
    waypoint_file_path_("../scripts/waypoints.csv"),
    map_frame_("map"),
    base_link_frame_("base_link"),
    reachedGoal(true),
    nh_()


{
    //ros::NodeHandle nhLocal("~");
    //loadQueue(); 
    // add dummy waypoint to be tossed outat start
    
    dummy_pose.header.stamp = ros::Time::now();
    dummy_pose.header.frame_id =  "" + map_frame_;
    dummy_pose.pose.position.x = 0;
    dummy_pose.pose.position.y = 0;
    dummy_pose.pose.position.z = 0;
    dummy_pose.pose.orientation.x = 0;
    dummy_pose.pose.orientation.y = 0;
    dummy_pose.pose.orientation.z = 0;
    dummy_pose.pose.orientation.w = 0;
    waypoint_queue_.push_back(dummy_pose);
}
void MainNode::saveWaypointCallback(const std_msgs::Empty::ConstPtr& msg){
        // save current pose to queue
    
    geometry_msgs::PoseStamped pose_stamped;
    tf::StampedTransform transform;
    try{
        listener_.lookupTransform(map_frame_, base_link_frame_, ros::Time(0), transform);
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id =  "" + map_frame_;

        pose_stamped.pose.position.x = transform.getOrigin().getX();
        pose_stamped.pose.position.y = transform.getOrigin().getY();
        pose_stamped.pose.position.z = transform.getOrigin().getZ();

        pose_stamped.pose.orientation.x = transform.getRotation().getX();
        pose_stamped.pose.orientation.y = transform.getRotation().getY();
        pose_stamped.pose.orientation.z = transform.getRotation().getZ();
        pose_stamped.pose.orientation.w = transform.getRotation().getW();
/*
        csv_file.open("../scripts/waypoints.csv", std::ios::out | std::ios::app); // open file in append mode
        if(csv_file.is_open()){
            csv_file << pose_stamped.header.stamp << "," << pose_stamped.header.frame_id << ","
                     << pose_stamped.pose.position.x << "," << pose_stamped.pose.position.y << "," 
                     << pose_stamped.pose.position.z << "," << pose_stamped.pose.orientation.x << ","
                     << pose_stamped.pose.orientation.y << "," << pose_stamped.pose.orientation.z << ","
                     << pose_stamped.pose.orientation.w << "\n";
            csv_file.close();
            ROS_INFO_STREAM("Waypoint saved");
        }
        else{
            ROS_ERROR_STREAM("Failed to open CSV file");
        }
*/
    }catch(tf::TransformException ex){
        ROS_WARN_STREAM(" cant get transform");
    }
    ROS_INFO_STREAM("saving current pose to queue");
    waypoint_queue_.push_back(pose_stamped);
        // Save the waypoints to file
    //std::ofstream ofs(waypoint_file_path_, std::ios::out | std::ios::trunc);
    
    
}
void MainNode::clickedPointCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    // save current pose to queue
    
    geometry_msgs::PoseStamped clicked_pose;
    clicked_pose.header.stamp = ros::Time::now();
    clicked_pose.header.frame_id =  "" + map_frame_;
    clicked_pose.pose.position.x = msg->pose.pose.position.x;
    clicked_pose.pose.position.y = msg->pose.pose.position.y;
    clicked_pose.pose.position.z = msg->pose.pose.position.z;
    clicked_pose.pose.orientation.x = msg->pose.pose.orientation.x;
    clicked_pose.pose.orientation.y = msg->pose.pose.orientation.y;
    clicked_pose.pose.orientation.z = msg->pose.pose.orientation.z;
    clicked_pose.pose.orientation.w = msg->pose.pose.orientation.w;
    waypoint_queue_.push_back(clicked_pose);
    ROS_INFO_STREAM("Waypoint saved from Rviz"<< clicked_pose.pose.position.x << clicked_pose.pose.position.y << clicked_pose.pose.position.z);
    
    
}

void MainNode::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    // Check if the status message is not empty
    if (!msg->status_list.empty()) {
        // Get the status of the first goal in the list (assuming only one goal is being executed at a time)
        const auto& status = msg->status_list.front();
        
        // Check if the status is 'aborted'
        if (status.status == actionlib_msgs::GoalStatus::ABORTED) {
            ROS_WARN("Navigation goal aborted!");
            //add dummy pose to send current goal again
            waypoint_queue_.push_back(dummy_pose);
            reachedGoal = true;
        }
    }
}
//toggle nav state
void MainNode::toggleNavigationCallback(const std_msgs::Empty::ConstPtr& msg){
    //switch navigation state
    if (navigation_state == false){
        ROS_INFO_STREAM("Navigation state = true");
        navigation_state=true;
    }
    else{
        navigation_state=false;
        // publish to cancel waypoint
        std_msgs::Empty cancel;
        cancel_waypoint.publish(cancel);
        ROS_INFO_STREAM("Navigation stopped");
    }
}
// start navigation
/*
void MainNode::startNavigationCallback(const std_msgs::Empty::ConstPtr& msg){
    //switch navigation state
    navigation_state = true;

}
*/

void MainNode::loadQueue(){
    // parse file for waypoints and add to queue
    std::ifstream ifs(waypoint_file_path_);
    if (ifs.is_open()) {
        std::string line;
        while (std::getline(ifs, line)) {
            std::stringstream ss(line);
            std::string item;
            std::vector<std::string> tokens;
            while (std::getline(ss, item, ',')) {
                tokens.push_back(item);
            }
            if (tokens.size() == 4) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "" + map_frame_;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.pose.position.x = std::stod(tokens[0]);
                pose_stamped.pose.position.y = std::stod(tokens[1]);
                pose_stamped.pose.orientation.z = std::stod(tokens[2]);
                pose_stamped.pose.orientation.w = std::stod(tokens[3]);
                waypoint_queue_.push_back(pose_stamped);
            }
        }
        ROS_INFO_STREAM("Loaded " << waypoint_queue_.size() << " waypoints from file " << waypoint_file_path_);
        ifs.close();
    } else {
        ROS_WARN_STREAM("Failed to open file " << waypoint_file_path_ << " for reading");
    }

}

void MainNode::currentPoseLoop(){
    geometry_msgs::PoseStamped current_pose;
    
    tf::StampedTransform transform;
    try{
        listener_.lookupTransform(map_frame_, base_link_frame_, ros::Time(0), transform);
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id =  "" + map_frame_;

        current_pose.pose.position.x = transform.getOrigin().getX();
        current_pose.pose.position.y = transform.getOrigin().getY();
        current_pose.pose.position.z = transform.getOrigin().getZ();

        current_pose.pose.orientation.x = transform.getRotation().getX();
        current_pose.pose.orientation.y = transform.getRotation().getY();
        current_pose.pose.orientation.z = transform.getRotation().getZ();
        current_pose.pose.orientation.w = transform.getRotation().getW();
        goal_pose = waypoint_queue_.front();
        float diff_x = abs(current_pose.pose.position.x - goal_pose.pose.position.x);
        float diff_y = abs(current_pose.pose.position.y - goal_pose.pose.position.y);
        distanceToGoal = QUADRATIC(diff_x,diff_y);
        if (distanceToGoal < 1.0){
            reachedGoal = true;
        }
        ROS_INFO_STREAM("distance to goal: " << distanceToGoal);
    }catch
    (tf::TransformException ex){
        ROS_WARN_STREAM(" cant get transform");
    }

    goal_pose = waypoint_queue_.front();
    float diff_x = abs(current_pose.pose.position.x - goal_pose.pose.position.x);
    float diff_y = abs(current_pose.pose.position.y - goal_pose.pose.position.y);
    distanceToGoal = QUADRATIC(diff_x,diff_y);
    if (distanceToGoal < 0.5){
        reachedGoal = true;
    }
    ROS_INFO_STREAM("distance to goal: " << distanceToGoal);
} 

int MainNode::run()
{
    ROS_INFO("waypoint controller node active");
    // connect subscribers
    save_waypoint_sub_ = nh_.subscribe("save_waypoint", 1, &MainNode::saveWaypointCallback, this);
    start_nav_sub_ = nh_.subscribe("start_navigation", 1, &MainNode::toggleNavigationCallback, this);
    clicked_point_sub_ = nh_.subscribe("initialpose", 1, &MainNode::clickedPointCallback, this);
    status_sub = nh_.subscribe("/move_base/status", 10, &MainNode::statusCallback,this);
    cancel_waypoint = nh_.advertise<std_msgs::Empty>("cancel_waypoint", 1);
    //stop_nav_sub_ = nh_.subscribe("stop_navigation", 1, &MainNode::stopNavigationCallback, this);
    //init MoveBaseClient
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    //load waypoints from file
    //loadQueue();
    //start loop
    ros::Rate loop_rate(10);
    while(ros::ok()){
        //check if navigation is active
        if (navigation_state == true){
            //check if waypoint queue is empty
            if (waypoint_queue_.empty()){
                ROS_INFO_STREAM("Waypoint queue empty");
                navigation_state = false;
            }
            else{
                //check if robot has reached goal
                if (reachedGoal == true){
                    //remove waypoint from queue
                    waypoint_queue_.erase(waypoint_queue_.begin());
                    //check if waypoint queue is empty
                    if (waypoint_queue_.empty()){
                        ROS_INFO_STREAM("Waypoint queue empty");
                        navigation_state = false;
                    }
                    else{
                        //send next waypoint to move_base
                        goal_pose = waypoint_queue_.front();
                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose = goal_pose;
                        goal.target_pose.header.frame_id = "" + map_frame_;
                        goal.target_pose.header.stamp = ros::Time::now();
                        ROS_INFO_STREAM("Sending goal");
                        ac.sendGoal(goal);
                        reachedGoal = false;
                    }
                }
                else{
                    //check if robot is close to goal
                    currentPoseLoop();
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");

  MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  //signal(SIGINT, mySigintHandler);

  return node.run();
}


