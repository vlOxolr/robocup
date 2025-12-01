#ifndef TIAGO_MOVE_H_
#define TIAGO_MOVE_H_

#include <string>
#include <vector>
#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

namespace tiago_move
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class Controller
  {
    private:
      // MoveIt 规划器（原样保留）
      moveit::planning_interface::MoveGroupInterface body_planner_;

    public:
      Controller();
      virtual ~Controller();

      bool initialize(ros::NodeHandle& nh);

      move_base_msgs::MoveBaseGoal createGoal(std::vector<double>& goal);

      std::vector<double> target_position;

      std::vector<double> target_pose;

      MoveBaseClient ac;


      int move_arm(std::vector<double>& goal);
  };
}

#endif
