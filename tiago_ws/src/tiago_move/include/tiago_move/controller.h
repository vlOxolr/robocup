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
  // Creates a typedef for a SimpleActionClient that communicates with actions that adhere to the MoveBaseAction action interface.
  // typedef /*#>>>>TODO: ACTION CLIENT TYPE*/</*#>>>>TODO: ACTION NAME*/> MoveBaseClient;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class Controller
  {
    private:
      //#>>>>TODO:Exercise4 create a MoveGroupInterface instance with name "body_planner_" for MoveGroup "armtorso"
      moveit::planning_interface::MoveGroupInterface body_planner_;

    public:
      Controller();
      virtual ~Controller();

      bool initialize(ros::NodeHandle& nh);

      move_base_msgs::MoveBaseGoal createGoal(std::vector<double>&);

      // Ex4: 关节/末端执行器目标（x y z r p y）
      std::vector<double> target_pose;

      //#>>>>TODO:Exercise3 Creates a SimpleActionClient that communicate with the move_base action server.
      MoveBaseClient ac;

      //#>>>>TODO:Exercise3 create a vector called nav_goals to store the waypoints
      std::vector<std::vector<double>> nav_goals;

      // Uncomment the function for Exercise 4
      // int move_arm(std::vector<double>&);
      int move_arm(std::vector<double>&);
  };
}

#endif
