
#include <opstop/ipopt_problem.hpp>
#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
// --
#include <chrono> // for high_resolution_clock
#include <cmath>
#include <gsplines/Functions/ElementalFunctions.hpp>
#include <gsplines/Interpolator.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <sensor_msgs/JointState.h>
namespace opstop_ros {

const std::string LOGNAME("OpStopControlActionWrapper");

const std::vector<std::string> smoothness_measures_available = {
    "jerk_l2_max", "acceleration_max"};

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_name, const std::string &_fjta_name,
    double _control_step, double _optimization_window, double _network_window,
    double _alpha, std::string _smoothness_measure,
    const pinocchio::Model &_model, std::size_t _nglp)
    : gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper(
          _name, _fjta_name, _control_step),
      optimization_window_milliseconds_(_optimization_window),
      network_window_milliseconds_(_network_window), alpha_(_alpha),
      model_(_model), nglp_(_nglp),
      smoothness_measure_(std::move(_smoothness_measure)) {

  joint_state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      "joint_states", 1000,
      [this](const sensor_msgs::JointState::ConstPtr &msg) {
        mutex_.lock();
        joint_state_ = *msg;
        mutex_.unlock();
      });
}

FollowJointTrajectoryActionWrapper::~FollowJointTrajectoryActionWrapper() =
    default;

void FollowJointTrajectoryActionWrapper::action_callback() {

  // 1. accept the goal from the action server
  const auto &goal = action_server_->acceptNewGoal();
  desired_motion_start_time_ = goal->gspline.header.stamp;
  // 2. Forward the goal.
  forward_goal(*goal);

  // --------------------------------------------------------------------------
  // 3. Check correctness of the model and build the pinocchio-model-consistent
  //    trajectory
  // --------------------------------------------------------------------------

  // 3.1 Check that we have all joints on the joint state publisher
  mutex_.lock();
  sensor_msgs::JointState js = joint_state_;
  mutex_.unlock();

  // 3.1.1 Check that we have the same number of joints in the model and in the
  //       joint state publisher
  if (js.name.size() != model_.nq) {
    original_trajectory_ = nullptr;
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Cannot figure out model joint positions");
    return;
  }

  // 3.1.2 Check that all the required joints are published and build the map
  //       between model joint indexes and joint state publisher indexes.
  std::unordered_map<long, long> model_joint_index_to_joint_state_joint_index;
  long index = 0;
  for (const auto &jname : js.name) {
    if (!model_.existJointName(jname)) {
      original_trajectory_ = nullptr;
      ROS_ERROR_STREAM_NAMED(LOGNAME,
                             "Joint name '" << jname << "' is not in model");
      return;
    }

    model_joint_index_to_joint_state_joint_index[static_cast<long>(
        model_.idx_qs[model_.getJointId(jname)])] = index;
    ROS_INFO_STREAM_NAMED(LOGNAME,
                          "Registering joint '"
                              << jname << "' id in model "
                              << model_.idx_qs[model_.getJointId(jname)]
                              << " id in joint state " << index);
    index++;
  }

  // --------------------------------------------------------------------------
  // 4. Build the pinocchio-model-consistent trajectory
  // --------------------------------------------------------------------------

  // 4.1 Get the waypoints of the trajectory
  Eigen::MatrixXd model_consistent_waypoints(
      goal->gspline.gspline.number_of_intervals + 1, js.name.size());

  for (const auto &jointMap : model_joint_index_to_joint_state_joint_index) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "a jointMap.first " << jointMap.first
                                                        << " jointMap.second "
                                                        << jointMap.second);
    model_consistent_waypoints.col(jointMap.first).array() =
        js.position[jointMap.second];
  }
  // 4.2 Get the map between the goal trajectory and the model joint indexes

  std::unordered_map<long, long> model_joint_index_to_gspline_joint_index;
  index = 0; // IMPORTANT!!! we used index above, so we need to set this.
  for (const auto &jname : goal->gspline.name) {
    if (!model_.existJointName(jname)) {
      original_trajectory_ = nullptr;
      ROS_ERROR_STREAM_NAMED(LOGNAME,
                             "Joint name '" << jname << "' is not in model");
      return;
    }
    model_joint_index_to_gspline_joint_index[static_cast<long>(
        model_.idx_qs[model_.getJointId(jname)])] = index;
    index++;
  }
  // 4.3 Get the waypoints of the original gspline
  // 4.3.1 Get the original gspline
  original_trajectory_ =
      gsplines_ros::gspline_msg_to_gspline(goal->gspline.gspline).move_clone();
  original_trajectory_goal_names_ = goal->gspline.name;

  // 4.3.2 Get the original gspline's waypoints
  auto gspline_actuated_waypoints = original_trajectory_->get_waypoints();
  // 4.3.4 Get the map between the model joint indexes and the original
  //       trajectory indexes
  for (const auto &jointMap : model_joint_index_to_gspline_joint_index) {
    model_consistent_waypoints.col(jointMap.first) =
        gspline_actuated_waypoints.col(jointMap.second);
  }

  // 4.4 Get the model consisten trajectory from interpolation
  pinocchio_model_consistent_trajectory_ =
      gsplines::interpolate(original_trajectory_->get_interval_lengths(),
                            model_consistent_waypoints,
                            original_trajectory_->get_basis())
          .move_clone();

  ROS_INFO_STREAM_NAMED(
      LOGNAME,
      "desired start time  "
          << (desired_motion_start_time_.toSec() -
              std::trunc(desired_motion_start_time_.toSec() / 100.0) * 100.0));
}

/// -----------------------------------------------------------------
/// --------------- PREEMPTION ACTION ------------------------------
/// -----------------------------------------------------------------
void FollowJointTrajectoryActionWrapper::preemption_action() {

  if (!original_trajectory_ || !pinocchio_model_consistent_trajectory_) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Empty trajectory, calling default stop");
    action_client_->cancelGoal();
    return;
  }
  // 1. Get the current time
  preemption_time_ = ros::Time::now();

  // 2. compute the time when the original trajectory should stop
  double ti =
      (preemption_time_ - desired_motion_start_time_).toSec() +
      (optimization_window_milliseconds_ + network_window_milliseconds_) *
          1.0e-3;

  std_msgs::Header stop_motion_header;
  stop_motion_header.stamp = desired_motion_start_time_ + ros::Duration(ti);

  ROS_INFO_STREAM_NAMED(LOGNAME, "Preemption request at "
                                     << preemption_time_.toSec()
                                     << ", time from start to stop ti =  " << ti
                                     << "stop trajectory starts at "
                                     << stop_motion_header.stamp.toSec());

  // 3. compute the time when the original trajectory should stop
  auto optimization_start_time = std::chrono::high_resolution_clock::now();

  double total_window_milliseconds =
      (optimization_window_milliseconds_ + network_window_milliseconds_);

  static int aux_int = 0;

  ROS_WARN_STREAM("model torque limits \n" << model_.effortLimit);

  if (ti >= pinocchio_model_consistent_trajectory_->get_domain().second) {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME,
        "Contradictory info stopping trj. time ti = "
            << ti << " is larger than the trajectory excution time "
            << pinocchio_model_consistent_trajectory_->get_domain().second);
    action_client_->cancelGoal();
    return;
  }
  gsplines::functions::FunctionExpression diffeo =
      opstop::minimum_time_bounded_acceleration(
          *pinocchio_model_consistent_trajectory_, ti, alpha_, model_, nglp_);

  if (aux_int % 2 == 0) {
    ROS_INFO_STREAM("+++++++++++++++++++++++++++++++++++++\n ---- minimizing "
                    "with acceleration ----- \n ");
  } else {

    ROS_INFO_STREAM("+++++++++++++++++++++++++++++++++++++\n ---- minimizing "
                    "with jerk l2 ----- \n ");
  }
  aux_int++;
  double end_time = diffeo.get_domain().second;

  // get the stopping trajectory
  gsplines::functions::FunctionExpression stop_trj =
      original_trajectory_->compose(diffeo).compose(
          gsplines::functions::Identity({ti, end_time}));

  // get the new goal for the follow joint trajectory controller
  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::function_to_follow_joint_trajectory_goal(
          stop_trj, original_trajectory_goal_names_,
          ros::Duration(get_control_step()), stop_motion_header);

  auto optimization_complete_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> computation_time_millisecods =
      optimization_complete_time - optimization_start_time;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Optimization time [ms]: "
                                     << computation_time_millisecods.count());

  /// if it tool too much time to optimize call emergency stop
  if (computation_time_millisecods.count() >
      optimization_window_milliseconds_) {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "optimizaion took too much time, canceling goal");
    action_client_->cancelGoal();
  } else {
    action_client_->sendGoal(
        goal_to_forward,
        [this](auto &&PH1, auto &&PH2) {
          done_action(std::forward<decltype(PH1)>(PH1),
                      std::forward<decltype(PH2)>(PH2));
        },
        [this] { active_action(); },
        [this](auto &&PH1) {
          feedback_action(std::forward<decltype(PH1)>(PH1));
        });
  }
}

void FollowJointTrajectoryActionWrapper::feedback_action(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_result) {

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper::
      feedback_action(_result);
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
