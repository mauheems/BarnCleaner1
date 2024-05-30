#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <custom_msgs/ObjectLocationArray.h>
#include <sensor_msgs/BatteryState.h>
#include <vector>
#include <geometry_msgs/PoseWithCovariance.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}

class MissionPlanner {
public:
    MissionPlanner() 
    {
        // Initialize subscribers
        sub_obj_track = nh.subscribe("/tracker/feces_locations", 10, &MissionPlanner::objTrackCallback, this);
        sub_own_location = nh.subscribe("/acml_pose", 10, &MissionPlanner::ownLocationCallback, this);
        sub_battery = nh.subscribe("/mirte/power/power_watcher", 10, &MissionPlanner::batteryCallback, this);

        // Initialize publisher (assuming a topic name, since it is missing in the original code)
        pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/goal_topic", 10);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_obj_track;
    ros::Subscriber sub_own_location;
    ros::Subscriber sub_battery;
    ros::Publisher pub_goal;

    std::vector<geometry_msgs::PoseStamped> path;  // from the global path planner
    std::vector<custom_msgs::ObjectLocationArray> faeces_location;
    geometry_msgs::PoseWithCovariance robot_location;
    std::vector<geometry_msgs::PoseStamped> path_to_follow;
    std::string status;
    geometry_msgs::PoseStamped current_goal;

    void objTrackCallback(const custom_msgs::ObjectLocationArray::ConstPtr& msg) {
        // Placeholder function for object tracking callback
    }

    void ownLocationCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& data) {
        // Placeholder function for own location callback
    }

    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
        // Placeholder function for battery state callback
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_planner");
    MissionPlanner mission_planner;
    ros::spin();
    return 0;
}