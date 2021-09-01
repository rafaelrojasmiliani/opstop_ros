
#include <chrono> // for high_resolution_clock
#include <gsplines/Functions/ElementalFunctions.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <math.h>
#include <opstop/ipopt_problem.hpp>
#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
namespace opstop_ros {

void FollowJointTrajectoryActionWrapper::action_callback() {

  const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal =
      action_server_->acceptNewGoal();

  desired_motion_start_time_ = goal->gspline.header.stamp;

  trajectory_ =
      gsplines_ros::gspline_msg_to_gspline(goal->gspline.gspline).move_clone();

  ROS_INFO("desired start time %+.3lf",
           desired_motion_start_time_.toSec() -
               std::trunc(desired_motion_start_time_.toSec() / 100.0) * 100.0);

  current_goal_names_ = goal->gspline.name;

  forward_goal(goal);
}

void FollowJointTrajectoryActionWrapper::prehemption_action() {

  prehemption_time_ = ros::Time::now();

  double ti = (prehemption_time_ - desired_motion_start_time_).toSec() +
              (optimization_window_milisec_ + network_window_milisec_) * 1.0e-3;

  std_msgs::Header stop_motion_header;
  stop_motion_header.stamp = desired_motion_start_time_ + ros::Duration(ti);

  ROS_INFO("Prehemtion request at %.3lf, time from start to stop ti = %.3lf, "
           "stop trajectry starts at %.3lf",
           prehemption_time_.toSec(), ti, stop_motion_header.stamp.toSec());

  auto optimization_start_time = std::chrono::high_resolution_clock::now();

  double total_window_milliseconds =
      (optimization_window_milisec_ + network_window_milisec_);

  gsplines::functions::FunctionExpression diffeo =
      opstop::minimum_time_bouded_acceleration(*trajectory_, ti, 2.5);

  double end_time = diffeo.get_domain().second;
  gsplines::functions::FunctionExpression stop_trj =
      trajectory_->compose(diffeo).compose(
          gsplines::functions::Identity({ti, end_time}));

  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::function_expression_to_follow_joint_trajectory_goal(
          stop_trj, current_goal_names_, ros::Duration(get_control_step()),
          stop_motion_header);

  auto optimization_complete_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> computation_time_millisecods =
      optimization_complete_time - optimization_start_time;

  if (computation_time_millisecods.count() > optimization_window_milisec_) {
    ROS_ERROR("optimizaion took too much time, canceling goal");
    action_client_->cancelGoal();
  } else {
    ROS_ERROR("goal time stamp %lf",
              goal_to_forward.trajectory.header.stamp.toSec());
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
  last_update_time_ = ros::Time::now();
  /*
  ROS_INFO("%+17.7e %+17.7e %+17.7e ", _result->actual.time_from_start.toSec(),
           _result->desired.time_from_start.toSec(),
           _result->error.time_from_start.toSec());
*/
}
void FollowJointTrajectoryActionWrapper::active_action() {
  if (desired_motion_start_time_.toSec() == 0) {
    desired_motion_start_time_ = ros::Time::now();
  }
  action_accepted_time_ = ros::Time::now();
}
} // namespace opstop_ros
