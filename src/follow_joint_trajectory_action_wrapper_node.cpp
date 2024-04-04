#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>
#include <pinocchio/parsers/urdf.hpp>
// --
#include <gsplines_ros/gsplines_ros.hpp>
// --
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

template <typename T>
void my_get_param(T &_val, ros::NodeHandle &_nh, const std::string &_param_name,
                  const XmlRpc::XmlRpcValue::Type &_xml_type,
                  bool print_value = true) {

  XmlRpc::XmlRpcValue xmlval;
  if (_nh.getParam(_param_name, xmlval) and xmlval.getType() == _xml_type) {
    try {
      _val = static_cast<T>(xmlval);
      if (print_value) {
        ROS_INFO_STREAM(_param_name << " is set " << _val);
      } else {

        ROS_INFO_STREAM(_param_name << " is set correctly");
      }
    } catch (XmlRpc::XmlRpcException &) {
      if (print_value) {
        ROS_INFO_STREAM(_param_name << " (exeption) is set to default "
                                    << _val);
      } else {
        ROS_INFO_STREAM(_param_name << " is set to default");
      }
    }
  } else {
    if (print_value) {
      ROS_INFO_STREAM(_param_name << " is set to default " << _val);
    } else {
      ROS_INFO_STREAM(_param_name << " is set to default");
    }
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "wrapper");
  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue xmlval;
  double control_step = 0.01;
  double alpha = 5.0;
  int optimization_window_milisec = 100.0;
  int network_window_milisec = 50.0;
  bool has_time_feedback = true;
  std::string robot_description;
  std::string smoothness_measure;
  int nglp = 7; // number of gauss-lobatto points

  std::string action_name = "follow_joint_gspline";
  std::string target_action_ns = "pos_joint_traj_controller";

  my_get_param(control_step, nh_priv, "control_step",
               XmlRpc::XmlRpcValue::TypeDouble);

  my_get_param(action_name, nh_priv, "action_name",
               XmlRpc::XmlRpcValue::TypeString);

  my_get_param(target_action_ns, nh_priv, "target_action_ns",
               XmlRpc::XmlRpcValue::TypeString);

  my_get_param(optimization_window_milisec, nh_priv,
               "optimization_window_milliseconds",
               XmlRpc::XmlRpcValue::TypeInt);

  my_get_param(network_window_milisec, nh_priv, "network_window_milliseconds",
               XmlRpc::XmlRpcValue::TypeInt);

  my_get_param(alpha, nh_priv, "alpha", XmlRpc::XmlRpcValue::TypeDouble);

  my_get_param(robot_description, nh, "robot_description",
               XmlRpc::XmlRpcValue::TypeString, false);

  my_get_param(smoothness_measure, nh_priv, "smoothness_measure",
               XmlRpc::XmlRpcValue::TypeString);

  my_get_param(nglp, nh_priv, "nglp", XmlRpc::XmlRpcValue::TypeInt);

  if (nglp <= 0) {
    ROS_FATAL(                                                     // NOLINT
        "the number of gauss lobatto points cannot be negative!"); // NOLINT
    return 1;
  }

  if (not opstop_ros::FollowJointTrajectoryActionWrapper::is_measure_available(
          smoothness_measure)) {

    ROS_FATAL_STREAM("requested smoothness measure is not avaiable");
    return 1;
  }

  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(robot_description, model);
  opstop_ros::FollowJointTrajectoryActionWrapper wrapper(
      action_name, target_action_ns, control_step, optimization_window_milisec,
      network_window_milisec, alpha, smoothness_measure, model, nglp);
  ros::spin();

  return 0;
}
