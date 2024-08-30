
#include <opstop/ipopt_problem.hpp>
#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
// --
#include <chrono> // for high_resolution_clock
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <gsplines/Functions/ElementalFunctions.hpp>
#include <gsplines/Interpolator.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <opstop_ros/FollowJointTrajectoryActionWrapperDynamicReconfigureConfig.h>

#include <gsplines/Collocation/GaussLobattoLagrange.hpp>
#include <gsplines_msgs/JointGSpline.h>

#include <optional>
#include <sensor_msgs/JointState.h>
namespace opstop_ros {

const std::string LOGNAME("OpStopControlActionWrapper");

const std::vector<std::string> smoothness_measures_available = {
    "jerk_l2_max", "acceleration_max"};

std::optional<std::size_t> smoothness_measure_to_uint(const std::string &_in) {

  if (_in == smoothness_measures_available[0]) {
    return 0;
  }
  if (_in == smoothness_measures_available[1]) {
    return 1;
  };
  return std::nullopt;
}

class FollowJointTrajectoryActionWrapper::Impl {
public:
  ros::NodeHandle nh_;
  double optimization_window_milliseconds = 100;
  double network_window_milliseconds = 10;
  double alpha = 2.0;
  std::size_t nglp = 4;
  std::size_t smoothness_measure;
  using ConfigType =
      opstop_ros::FollowJointTrajectoryActionWrapperDynamicReconfigureConfig;

  dynamic_reconfigure::Server<ConfigType> server_;

  ros::Publisher stop_trj_approx_gspline_publisher_;

  Impl(double _optimization_window, double _network_window, double _alpha,
       std::size_t _nglp, std::string &_smoothness_measure)
      : nh_("~/opstop_optimizer"), server_(nh_),
        optimization_window_milliseconds(_optimization_window),
        network_window_milliseconds(_network_window), alpha(_alpha),
        nglp(_nglp),
        smoothness_measure(
            smoothness_measure_to_uint(_smoothness_measure).value()) {

    server_.setCallback([this](ConfigType &_cfg, uint32_t _level) {
      this->set_parameters(_cfg, _level);
    });

    ConfigType config;
    config.alpha = _alpha;
    config.optimization_window_milliseconds =
        static_cast<int>(_optimization_window);
    config.network_window_milliseconds = static_cast<int>(_network_window);
    config.alpha = _alpha;
    config.smoothness_measure = static_cast<int>(smoothness_measure);

    stop_trj_approx_gspline_publisher_ =
        nh_.advertise<gsplines_msgs::JointGSpline>(
            "computed_stop_gspline_approximation", 1000);
  }

  //   bool get_basis(typename GetBasisSrv::Request &req, // NOLINT
  //                  typename GetBasisSrv::Response &res) {

  //     (void)req;
  //     res.basis = gsplines_ros::basis_to_basis_msg(*this->basis);
  //     res.success = true; // NOLINT
  //     res.message = "";
  //     return true;
  //   }

  void set_parameters(ConfigType &_cfg, uint32_t level) {
    (void)level;

    optimization_window_milliseconds = _cfg.optimization_window_milliseconds;
    network_window_milliseconds = _cfg.network_window_milliseconds;
    alpha = _cfg.alpha;
    nglp = _cfg.nglp;
    smoothness_measure = _cfg.smoothness_measure;

    opstop::optimization::IpoptSolverOptions::set_option("linear_solver",
                                                         _cfg.linear_solver);
    opstop::optimization::IpoptSolverOptions::set_option(
        "jacobian_approximation", _cfg.jacobian_approximation);

    opstop::optimization::IpoptSolverOptions::set_option(
        "fast_step_computation", _cfg.fast_step_computation);

    opstop::optimization::IpoptSolverOptions::set_option("derivative_test",
                                                         _cfg.derivative_test);

    opstop::optimization::IpoptSolverOptions::set_option(
        "hessian_approximation", _cfg.hessian_approximation);

    opstop::optimization::IpoptSolverOptions::set_option("jac_c_constant",
                                                         _cfg.jac_c_constant);

    opstop::optimization::IpoptSolverOptions::set_option(
        "print_timing_statistics", _cfg.print_timing_statistics);

    opstop::optimization::IpoptSolverOptions::set_option(
        "dependency_detector", _cfg.dependency_detector);

    opstop::optimization::IpoptSolverOptions::set_option(
        "dependency_detection_with_rhs", _cfg.dependency_detection_with_rhs);

    opstop::optimization::IpoptSolverOptions::set_option("tol", _cfg.tol);

    opstop::optimization::IpoptSolverOptions::set_option("dual_inf_tol",
                                                         _cfg.dual_inf_tol);

    opstop::optimization::IpoptSolverOptions::set_option("constr_viol_tol",
                                                         _cfg.constr_viol_tol);

    opstop::optimization::IpoptSolverOptions::set_option("compl_inf_tol",
                                                         _cfg.compl_inf_tol);

    opstop::optimization::IpoptSolverOptions::set_option("print_level",
                                                         _cfg.print_level);
  }
};

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_name, const std::string &_fjta_name,
    double _control_step, double _optimization_window, double _network_window,
    double _alpha, std::string &_smoothness_measure,
    const pinocchio::Model &_model, std::size_t _nglp)
    : gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper(
          _name, _fjta_name, _control_step),
      model_(_model), m_impl(new Impl(_optimization_window, _network_window,
                                      _alpha, _nglp, _smoothness_measure)) {

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
  // 3. Check correctness of the model and build the
  // pinocchio-model-consistent
  //    trajectory
  // --------------------------------------------------------------------------

  // 3.1 Check that we have all joints on the joint state publisher
  mutex_.lock();
  sensor_msgs::JointState js = joint_state_;
  mutex_.unlock();

  // 3.1.1 Check that we have the same number of joints in the model and in
  // the
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
                          "Associating joint from message with id '"
                              << jname << "' to joint in pinocchio model "
                              << model_.idx_qs[model_.getJointId(jname)]
                              << " with id in joint state " << index);
    index++;
  }

  // --------------------------------------------------------------------------
  // 4. Build the pinocchio-model-consistent trajectory
  // --------------------------------------------------------------------------

  // 4.1 Get the waypoints of the trajectory
  Eigen::MatrixXd model_consistent_waypoints(
      goal->gspline.gspline.number_of_intervals + 1, js.name.size());

  for (const auto &jointMap : model_joint_index_to_joint_state_joint_index) {
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
  double ti = (preemption_time_ - desired_motion_start_time_).toSec() +
              (m_impl->optimization_window_milliseconds +
               m_impl->network_window_milliseconds) *
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

  double total_window_milliseconds = (m_impl->optimization_window_milliseconds +
                                      m_impl->network_window_milliseconds);

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

  auto diffeo = opstop::minimum_time_bounded_acceleration(
      *pinocchio_model_consistent_trajectory_, ti, m_impl->alpha, model_,
      m_impl->nglp);

  ROS_ERROR_STREAM_NAMED(LOGNAME, "_-------------------------------");

  if (!diffeo.has_value()) {
    action_client_->cancelGoal();
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to compute the smooth trajectory");
    return;
  }

  // if (aux_int % 2 == 0) {
  //   ROS_INFO_STREAM("+++++++++++++++++++++++++++++++++++++\n ---- minimizing
  //   "
  //                   "with acceleration ----- \n ");
  // } else {

  //   ROS_INFO_STREAM("+++++++++++++++++++++++++++++++++++++\n ---- minimizing
  //   "
  //                   "with jerk l2 ----- \n ");
  // }
  // aux_int++;
  double end_time = diffeo.value().get_domain().second;

  // get the stopping trajectory
  gsplines::functions::FunctionExpression stop_trj =
      original_trajectory_->compose(diffeo.value())
          .compose(gsplines::functions::Identity({ti, end_time}));

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
      m_impl->optimization_window_milliseconds) {
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
  auto approx_stop_trajectory =
      gsplines::collocation::GaussLobattoLagrangeSpline::approximate(stop_trj,
                                                                     5, 5);

  m_impl->stop_trj_approx_gspline_publisher_.publish(
      gsplines_ros::gspline_to_joint_gspline_msg(
          approx_stop_trajectory, original_trajectory_goal_names_));
}

void FollowJointTrajectoryActionWrapper::feedback_action(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_result) {

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper::
      feedback_action(_result);
  /*
  ROS_INFO("%+17.7e %+17.7e %+17.7e ",
  _result->actual.time_from_start.toSec(),
           _result->desired.time_from_start.toSec(),
           _result->error.time_from_start.toSec());
*/
}
void FollowJointTrajectoryActionWrapper::active_action() {
  if (desired_motion_start_time_.toSec() == 0) {
    desired_motion_start_time_ = ros::Time::now();
  }
}
} // namespace opstop_ros
