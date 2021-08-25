#ifndef FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#define FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H
#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>
namespace opstop_ros {

class FollowJointTrajectoryActionWrapper
    : public gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper {
private:
public:
  FollowJointTrajectoryActionWrapper(
      const FollowJointTrajectoryActionWrapper &) = delete;
  FollowJointTrajectoryActionWrapper &
  operator=(const FollowJointTrajectoryActionWrapper &) = delete;

  using gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper::
      FollowJointTrajectoryActionWrapper;

  virtual ~FollowJointTrajectoryActionWrapper() = default;
  void prehemption_action() override;

  void action_callback(
      const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal) override;
};
} // namespace opstop_ros

#endif /* FOLLOW_JOINT_TRAJECTORY_ACTION_WRAPPER_H */
