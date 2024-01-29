////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    motion_control_handle.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/detail/interactive_marker_feedback__struct.hpp"
#include <end_effector_controller/end_effector_control.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>



namespace end_effector_controller {

EndEffectorControl::EndEffectorControl() {}

EndEffectorControl::~EndEffectorControl() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, m_joint_names, hardware_interface::HW_IF_POSITION, m_joint_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(),
                 hardware_interface::HW_IF_POSITION,
                 m_joint_handles.size());
    return CallbackReturn::ERROR;
  }

  m_current_pose = getEndEffectorPose();
  starting_position = m_current_pose.pose.position;

  m_ft_sensor_wrench(0) = 0.0;
  m_ft_sensor_wrench(1) = 0.0;
  m_ft_sensor_wrench(2) = 0.0;

  m_target_wrench(0) = 0.0;
  m_target_wrench(1) = 0.0;
  m_target_wrench(2) = 0.0;

  tr_counter = 0;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_joint_handles.clear();
  this->release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


controller_interface::return_type EndEffectorControl::update(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period)
{
  // Get the end effector pose 
  m_current_pose = getEndEffectorPose();

  // Give the new point to the end effector following a circle with a radius of 0.03m and a frequency of 0.2Hz if the force
  // if whithin a 10 percent range
  if (abs(m_ft_sensor_wrench[2]) <= 1.05 * abs(m_target_wrench[2]) && abs(m_ft_sensor_wrench[2]) >= 0.95 * abs(m_target_wrench[2]))
  {
    m_current_pose.pose.position.x = starting_position.x + 0.03 * sin(2 * M_PI * tr_counter / 16000) * tr_counter / 50000;
    m_current_pose.pose.position.y = starting_position.y + 0.06 * cos(2 * M_PI * tr_counter / 16000) * tr_counter / 50000;
    tr_counter ++;
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "tr_counter" << tr_counter);
    m_sinusoidal_force.wrench.force.z =  0.5 * sin(2 * M_PI * tr_counter / 2000) + 8.0;
    m_error_publisher->publish(m_sinusoidal_force);
  }
  else
  {
    m_current_pose.pose.position.x = starting_position.x + 0.03 * sin(2 * M_PI * tr_counter / 16000) * tr_counter / 50000;
    m_current_pose.pose.position.y = starting_position.y + 0.06 * cos(2 * M_PI * tr_counter / 16000) * tr_counter / 50000;
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "m_ft_sensor_wrench" << m_ft_sensor_wrench);
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "m_target_wrench" << m_target_wrench);
  } 
  // if (tr_counter >= 1000 ){
  //   m_current_pose.pose.position.x = starting_position.x + 0.04 * cos(2 * M_PI * (tr_counter-1000) / 8000);
  //   m_current_pose.pose.position.y = starting_position.y + 0.04 * sin(2 * M_PI * (tr_counter-1000) / 8000);
  //   tr_counter ++;
  // } else {
  //   m_current_pose.pose.position.x = starting_position.x;
  //   m_current_pose.pose.position.y = starting_position.y;
  //   tr_counter ++;
  // }
  
  // m_current_pose.pose.position.x = starting_position.x;
  // m_current_pose.pose.position.y = starting_position.y + (tr_counter / 20000.0);
  // tr_counter ++;
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "tr_counter: " << tr_counter);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "y pos: " << starting_position.y + tr_counter / 20000.0);
  // m_current_pose.pose.orientation.x = 1;
  // m_current_pose.pose.orientation.y = 0;
  // m_current_pose.pose.orientation.z = 0;
  // m_current_pose.pose.orientation.w = 0;
  // apply a proportional control to keep the end effector perpendicular to the surface
  m_current_pose.pose.orientation = setEndEffectorOrientation(m_current_pose.pose.orientation);
  // Publish marker pose
  m_current_pose.header.stamp    = get_node()->now();
  m_current_pose.header.frame_id = m_robot_base_link;
  m_pose_publisher->publish(m_current_pose);
  
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
EndEffectorControl::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration
EndEffectorControl::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size()); // Only position
  for (const auto& joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_init()
{

  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string> >("joints", std::vector<std::string>());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

  std::string robot_description = get_node()->get_parameter("robot_description").as_string();
  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
  {
    const std::string error = ""
                              "Failed to parse robot chain from urdf model. "
                              "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of the joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Publishers
  m_pose_publisher = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    get_node()->get_name() + std::string("/target_frame"), 10);

  m_error_publisher = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    std::string("/target_wrench"), 10);

  //Subscriber
  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"),
    10,
    std::bind(&EndEffectorControl::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"),
      10,
      std::bind(&EndEffectorControl::ftSensorWrenchCallback, this, std::placeholders::_1));

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));
  m_current_pose = getEndEffectorPose();
  Eigen::Quaterniond current_quat(m_current_pose.pose.orientation.w, m_current_pose.pose.orientation.x, m_current_pose.pose.orientation.y, m_current_pose.pose.orientation.z);
  Eigen::AngleAxisd current_aa(current_quat);
  tr_counter = 0;
  m_sinusoidal_force.wrench.force.x = 0.0;
  m_sinusoidal_force.wrench.force.y = 0.0;
  m_sinusoidal_force.wrench.force.z = 15.0;
  m_sinusoidal_force.wrench.torque.x = 0.0;
  m_sinusoidal_force.wrench.torque.y = 0.0;
  m_sinusoidal_force.wrench.torque.z = 0.0;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

geometry_msgs::msg::Quaternion EndEffectorControl::setEndEffectorOrientation(geometry_msgs::msg::Quaternion pos){
  
  // if no target force is not published the controller do nothing 
  if ( m_target_wrench[0] == 0.0 && m_target_wrench[1] == 0.0 && m_target_wrench[2] == 0.0 ){
    return pos;
  }
  // if no target force is not published the controller do nothing 
  if ( abs(m_ft_sensor_wrench[0]) <= 0.1 && abs(m_ft_sensor_wrench[1]) <= 0.1 && abs(m_ft_sensor_wrench[2]) <= 0.1 ){
    return pos;
  }
  // Normalize the vectors
  m_ft_sensor_wrench.normalize();
  m_target_wrench.normalize();

  // Calculate the cross product of the normalized vectors
  errorOrientation = m_ft_sensor_wrench.cross(m_target_wrench);

  // Calculate the dot product of the normalized vectors
  double cos_theta = m_ft_sensor_wrench.dot(m_target_wrench);
  if (cos_theta > 1.0)
  {
    cos_theta = 1.0;
  }
  if (cos_theta < -1.0)
  {
    cos_theta = -1.0;
  }

  // Calculate the angle between the vectors using the inverse cosine function
  double theta = std::acos(cos_theta);

  // Create a quaternion with the angle-axis representation
  Eigen::Quaterniond q_new(Eigen::AngleAxisd(theta, errorOrientation));

  // Define a previous quaternion rotation
  Eigen::Quaterniond q_current(pos.w, pos.x, pos.y, pos.z);

  // Multiply the two quaternions together to get the new rotation
  Eigen::Quaterniond q_final = q_new * q_current;
  q_final.normalize();

  geometry_msgs::msg::Point pt;
  pt.x = errorOrientation[0];
  pt.y = errorOrientation[1];
  pt.z = errorOrientation[2];

  // m_error_publisher->publish(pt);
  
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q_current" << q_current);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q_new" << q_new);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "m_ft_sensor_wrench" << m_ft_sensor_wrench);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "m_target_wrench" << m_target_wrench);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "errorOrientation" << errorOrientation);

  // Multiply the two quaternions together to get the new rotation
  Eigen::Quaterniond q_interp = q_current.slerp(1, q_final);

  pos.x = (double)q_interp.x();
  pos.y = (double)q_interp.y();
  pos.z = (double)q_interp.z();
  pos.w = (double)q_interp.w();

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q_interp" << q_interp);

  return  pos;
}

geometry_msgs::msg::PoseStamped EndEffectorControl::getEndEffectorPose()
{
  KDL::JntArray positions(m_joint_handles.size());
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    positions(i) = m_joint_handles[i].get().get_value();
  }

  KDL::Frame tmp;
  m_fk_solver->JntToCart(positions, tmp);

  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = tmp.p.x();
  current.pose.position.y = tmp.p.y();
  current.pose.position.z = tmp.p.z();
  tmp.M.GetQuaternion(current.pose.orientation.x,
                      current.pose.orientation.y,
                      current.pose.orientation.z,
                      current.pose.orientation.w);

  return current;
}

void EndEffectorControl::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;

  m_ft_sensor_wrench(0) = tmp[0];
  m_ft_sensor_wrench(1) = -tmp[1];
  m_ft_sensor_wrench(2) = tmp[2];
}

void EndEffectorControl::targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;

  m_target_wrench(0) = -tmp[0];
  m_target_wrench(1) = -tmp[1];
  m_target_wrench(2) = -tmp[2];
}

// void CartesianForceController::setFtSensorReferenceFrame(const std::string& new_ref)
// {
//   // Compute static transform from the force torque sensor to the new reference
//   // frame of interest.
//   m_new_ft_sensor_ref = new_ref;

//   // Joint positions should cancel out, i.e. it doesn't matter as long as they
//   // are the same for both transformations.
//   KDL::JntArray jnts(EndEffectorControl::m_ik_solver->getPositions());

//   KDL::Frame sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       sensor_ref,
//       m_ft_sensor_ref_link);

//   KDL::Frame new_sensor_ref;
//   Base::m_forward_kinematics_solver->JntToCart(
//       jnts,
//       new_sensor_ref,
//       m_new_ft_sensor_ref);

//   m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
// }

} // namespace cartesian_controller_handles


// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(end_effector_controller::EndEffectorControl,
                       controller_interface::ControllerInterface)
