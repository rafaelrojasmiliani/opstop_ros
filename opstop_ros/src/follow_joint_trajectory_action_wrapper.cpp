
#include <chrono> // for high_resolution_clock
#include <gsplines/Functions/ElementalFunctions.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <opstop/ipopt_problem.hpp>
#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
namespace opstop_ros {

void FollowJointTrajectoryActionWrapper::action_callback() {

  const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal =
      action_server_->acceptNewGoal();

  trajectory_ =
      gsplines_ros::msg_to_gspline(goal->gspline.gspline).move_clone();

  current_goal_names_ = goal->gspline.name;

  forward_goal(goal);
}

void FollowJointTrajectoryActionWrapper::prehemption_action() {

  prehemption_time_ = ros::Time::now();
  double ti = time_from_start_to_stop_ +
              (optimization_window_milisec_ + network_window_milisec_) * 1.0e-3;
  ROS_INFO(
      "Prehemtion requested at ti = %+14.7e, i.e %7.3lf%% of the trajectory",
      ti, ti / trajectory_->get_domain().second);
  auto start_time = std::chrono::high_resolution_clock::now();

  double total_window_milliseconds =
      (optimization_window_milisec_ + network_window_milisec_);

  ROS_INFO("optimizing");
  gsplines::functions::FunctionExpression diffeo =
      opstop::minimum_time_bouded_acceleration(*trajectory_, ti, 5);

  double end_time = diffeo.get_domain().second;
  ROS_INFO("ti = %+14.7e  ti+Ts = %+14.7e", ti, end_time);
  gsplines::functions::FunctionExpression stop_trj =
      trajectory_->compose(diffeo).compose(
          gsplines::functions::Identity({ti, end_time}));

  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::function_expression_to_follow_joint_trajectory_goal(
          stop_trj, current_goal_names_, ros::Duration(get_control_step()));

  auto completion_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> computation_time_millisecods =
      completion_time - start_time;

  ROS_INFO("total computation time %8.3lf",
           computation_time_millisecods.count());
  if (computation_time_millisecods.count() > optimization_window_milisec_) {
    ROS_ERROR("optimizaion done in to mmuch time, calleling goal");
    action_client_->cancelGoal();
  } else {
    goal_to_forward.trajectory.header.stamp =
        prehemption_time_ + ros::Duration(ti - time_from_start_to_stop_);
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
  ROS_INFO("%+17.7e %+17.7e %+17.7e ", _result->actual.time_from_start.toSec(),
           _result->desired.time_from_start.toSec(),
           _result->error.time_from_start.toSec());
}
} // namespace opstop_ros
