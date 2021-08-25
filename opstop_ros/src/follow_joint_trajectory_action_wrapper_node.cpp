#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <opstop_ros/follow_joint_trajectory_action_wrapper.hpp>

template <typename T>
void my_get_param(T &_val, ros::NodeHandle &_nh, const std::string &_param_name,
                  const XmlRpc::XmlRpcValue::Type &_xml_type) {

  XmlRpc::XmlRpcValue xmlval;
  if (_nh.getParam(_param_name, xmlval) and xmlval.getType() == _xml_type) {
    _val = static_cast<T>(xmlval);
  } else {
    ROS_INFO_STREAM(_param_name << " is set to default " << _val);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "wrapper");
  ros::NodeHandle node("~");
  XmlRpc::XmlRpcValue xmlval;
  double control_step = 0.01;
  double optimization_window_milisec = 100.0;
  double network_window_milisec = 50.0;

  std::string action_name = "follow_joint_gspline";
  std::string target_action_ns = "pos_joint_traj_controller";

  my_get_param(control_step, node, "control_step",
               XmlRpc::XmlRpcValue::TypeDouble);

  my_get_param(action_name, node, "action_name",
               XmlRpc::XmlRpcValue::TypeString);

  my_get_param(target_action_ns, node, "target_action_ns",
               XmlRpc::XmlRpcValue::TypeString);

  my_get_param(optimization_window_milisec, node,
               "optimization_window_milliseconds",
               XmlRpc::XmlRpcValue::TypeDouble);

  my_get_param(network_window_milisec, node, "network_window_milliseconds",
               XmlRpc::XmlRpcValue::TypeDouble);

  opstop_ros::FollowJointTrajectoryActionWrapper wrapper(
      action_name, target_action_ns, control_step, optimization_window_milisec,
      network_window_milisec);

  ros::spin();

  return 0;
}
