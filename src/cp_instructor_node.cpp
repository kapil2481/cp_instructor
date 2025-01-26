#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

void checkpointCallback(std::queue<geometry_msgs::PoseStamped> &queue, const geometry_msgs::PoseStamped &msg)
{
  ROS_INFO("GOT CHECKPOINT: [%f %f %f]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  queue.push(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cp_instructor_node");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  std::queue<geometry_msgs::PoseStamped> next_checkpoints;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/checkpoint", 1000, [&next_checkpoints](const boost::shared_ptr<const geometry_msgs::PoseStamped> &msg)
                                                                                 { checkpointCallback(next_checkpoints, *msg); });

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started.");

  ros::Rate loop_rate(10);  // Check for new messages at 10 Hz
  while (ros::ok())
  {
    ros::spinOnce(); // Call this to handle incoming messages

    if (!next_checkpoints.empty())
    {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = next_checkpoints.front();

      ROS_INFO("Sending goal to (%f, %f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z);

      ac.sendGoal(goal);
      next_checkpoints.pop();

      bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      }
      else
      {
        ROS_INFO("Action did not finish before the time out.");
        ac.cancelGoal();
      }
    }

    loop_rate.sleep(); // Sleep to maintain loop rate
  }

  return 0;
}
