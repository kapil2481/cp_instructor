#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include<queue>


void checkpointCallback(std::queue<geometry_msgs::PoseStamped>& queue, const geometry_msgs::PoseStamped& msg)
{
//   ROS_INFO("I heard: [%f %f %f]", msg.position.x, msg.position.y, msg.position.z);
  queue.push(msg);
}

int main (int argc, char **argv)
{
  
  ros::init(argc, argv, "cp_instructor_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  std::queue<geometry_msgs::PoseStamped> next_checkpoints;

  ros::NodeHandle n;
  ros::Subscriber pub = n.subscribe<geometry_msgs::PoseStamped>("checkpoint", 1000, [&next_checkpoints](geometry_msgs::PoseStamped& msg){ checkpointCallback(next_checkpoints, msg); });


  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time


  while(!next_checkpoints.empty()){
  
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose = next_checkpoints.front();

    ac.sendGoal(goal);

    next_checkpoints.pop();

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        ac.cancelGoal();
    }
  }
  //exit
  return 0;
}