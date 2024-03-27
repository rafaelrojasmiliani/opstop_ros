#ifndef FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#define FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#include <pinocchio/algorithm/model.hpp>
// --

#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <gsplines/Functions/FunctionBase.hpp>

#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>

#include <ros/duration.h>
#include <ros/time.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace opstop_ros {

/// This is a relay/repeater node which translates FollowJointGSplineAction
/// into a FollowJointTrajectoryAction able to smoothly stop a trajectory.
class FollowJointTrajectoryActionWrapper
    : public gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper {
private:
  std::unique_ptr<gsplines::functions::FunctionBase> trajectory_;
  double time_from_start_to_stop_ = 0.0;
  double expected_delay_;

  std::vector<std::string> current_goal_names_;

  double optimization_window_milisec_;
  double network_window_milisec_;
  double alpha_;

  std::size_t nglp_;

  ros::Time prehemption_time_;
  ros::Time last_update_time_;
  ros::Time action_accepted_time_;
  pinocchio::Model model_;

  std::string smoothness_measure_;

public:
  static const std::vector<std::string> smoothness_measures_available;

  static bool is_measure_available(const std::string &_str) {
    /*
  return std::find_if(smoothness_measures_available.cbegin(),
                      smoothness_measures_available.cend(),
                      [&_str](const std::string &_it) { return true; }) !=
         smoothness_measures_available.cend();*/
    return true;
  }

  // FollowJointTrajectoryActionWrapper(
  //     const FollowJointTrajectoryActionWrapper &) = delete;
  // FollowJointTrajectoryActionWrapper &
  // operator=(const FollowJointTrajectoryActionWrapper &) = delete;

  /**
   * @brief Constructs a FollowJointTrajectoryActionWrapper object.
   *
   * This constructor initializes a FollowJointTrajectoryActionWrapper
   * instance, setting up the necessary parameters for the ROS
   * FollowJointTrajectoryAction repeater. The wrapper acts as an intermediary,
   * adapting and forwarding joint trajectory commands to another
   * FollowJointTrajectoryAction server.
   *
   * @param _name Name of the action server provided by this wrapper
   * @param _fjta_name Name of the other FollowJointTrajectoryAction server
   * to which commands are forwarded.
   * @param _control_step Time step size (in seconds) for the control loop
   * execution.
   * @param _optimization_window Time window (in seconds) over which trajectory
   * optimization is performed.
   * @param _network_window Time window (in seconds) considered for compensating
   * network latencies and uncertainties.
   * @param _alpha Weight factor used in the optimization process, typically for
   * smoothness or other criteria.
   * @param _smoothness_measure String identifying the measure of smoothness (or
   * other optimization criteria) to be used.
   * @param _model Reference to a pinocchio::Model instance representing the
   * robot model used for trajectory calculations.
   * @param _nglp Number of Gauss-Lobatto points used for discretizing the
   * trajectory in the optimization process.
   */
  FollowJointTrajectoryActionWrapper(
      const std::string &_name, const std::string &_fjta_name,
      double _control_step, double _optimization_window, double _network_window,
      double _alpha, const std::string &_smoothness_measure,
      const pinocchio::Model &_model, std::size_t _nglp);

  /// Destructor
  ~FollowJointTrajectoryActionWrapper() override;

  /**
   * @brief Callback of the action server instantiated by this class. Here the
   * action goal forwarding is implemented.
   *
   */
  void action_callback() override;

  /// Action performed on prehmption request
  void prehemption_action() override;

  /**
   * @brief Action executed when the target client stars the execution of the
   * action
   *
   */
  void active_action() override;

  /**
   * @brief action performed when feedback from the target action is received.
   *
   * @param _result feedback from the target action.
   */
  void feedback_action(const control_msgs::FollowJointTrajectoryFeedbackConstPtr
                           &_result) override;
};
} // namespace opstop_ros

#endif /* FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H */
