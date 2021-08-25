
#include <chrono> // for high_resolution_clock
#include <gsplines_ros/gsplines_ros.hpp>
#include <opstop/ipopt_problem.hpp>
#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
namespace opstop_ros {

void FollowJointTrajectoryActionWrapper::action_callback(
    const gsplines_msgs::FollowJointGSplineGoalConstPtr &_goal) {

  trajectory_ =
      gsplines_ros::msg_to_gspline(_goal->gspline.gspline).move_clone();

  current_goal_names_ = _goal->gspline.name;

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper::
      action_callback(_goal);
}

void FollowJointTrajectoryActionWrapper::prehemption_action() {

  auto start_time = std::chrono::high_resolution_clock::now();
  double time_to_stop =
      time_from_start_to_stop_ +
      1e3 * (optimization_window_milisec_ + network_window_milisec_);

  gsplines::functions::FunctionExpression diffeo =
      opstop::minimum_time_bouded_acceleration(*trajectory_,
                                               time_from_start_to_stop_, 10);

  gsplines::functions::FunctionExpression stop_trj =
      trajectory_->compose(diffeo);

  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::function_expression_to_follow_joint_trajectory_goal(
          stop_trj, current_goal_names_, ros::Duration(get_control_step()));

  auto completion_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> computation_time =
      start_time - completion_time;

  if (computation_time.count() > optimization_window_milisec_) {
    ROS_ERROR("optimizaion done in to mmuch time, calleling goal");
    action_client_->cancelGoal();
  } else {
    action_client_->sendGoal(
        goal_to_forward,
        boost::bind(&FollowJointTrajectoryActionWrapper::done_action, this, _1,
                    _2),
        boost::bind(&FollowJointTrajectoryActionWrapper::active_action, this),
        boost::bind(&FollowJointTrajectoryActionWrapper::feedback_action, this,
                    _1));
  }
}

void FollowJointTrajectoryActionWrapper::feedback_action(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_result) {

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper::
      feedback_action(_result);
  time_from_start_to_stop_ = _result->desired.time_from_start.toSec();
}
} // namespace opstop_ros
