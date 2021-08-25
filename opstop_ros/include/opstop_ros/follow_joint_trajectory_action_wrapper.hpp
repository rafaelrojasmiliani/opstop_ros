#ifndef FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#define FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#include <gsplines/Functions/FunctionExpression.hpp>

#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>
namespace opstop_ros {

class FollowJointTrajectoryActionWrapper
    : public gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper {
private:
  std::unique_ptr<gsplines::functions::FunctionExpression> trajectory_;
  double time_from_start_to_stop_;
  double expected_delay_;

  std::vector<std::string> current_goal_names_;

  double optimization_window_milisec_;
  double network_window_milisec_;

public:
  FollowJointTrajectoryActionWrapper(
      const FollowJointTrajectoryActionWrapper &) = delete;
  FollowJointTrajectoryActionWrapper &
  operator=(const FollowJointTrajectoryActionWrapper &) = delete;

  FollowJointTrajectoryActionWrapper(const std::string &_name,
                                     const std::string &_fjta_name,
                                     double _control_step,
                                     double _optimization_window,
                                     double _network_window)
      : gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper(
            _name, _fjta_name, _control_step),
        trajectory_(nullptr), time_from_start_to_stop_(0.0),
        optimization_window_milisec_(_optimization_window),
        network_window_milisec_(_network_window) {}

  virtual ~FollowJointTrajectoryActionWrapper() = default;

  void prehemption_action() override;

  void action_callback(
      const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal) override;

  void feedback_action(const control_msgs::FollowJointTrajectoryFeedbackConstPtr
                           &_result) override;
};
} // namespace opstop_ros

#endif /* FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H */
