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

#include <end_effector_controller/end_effector_control.h>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/detail/interactive_marker_feedback__struct.hpp"

namespace end_effector_controller
{

EndEffectorControl::EndEffectorControl() {}

EndEffectorControl::~EndEffectorControl() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Get state handles.
  // if (!controller_interface::get_ordered_interfaces(
  //         state_interfaces_, m_joint_names, hardware_interface::HW_IF_POSITION, m_joint_handles))
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(),
  //                "Expected %zu '%s' state interfaces, got %zu.",
  //                m_joint_names.size(),
  //                hardware_interface::HW_IF_POSITION,
  //                m_joint_handles.size());
  //   return CallbackReturn::ERROR;
  // }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Velocity
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    m_joint_state_vel_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_VELOCITY,
                 m_joint_state_vel_handles.size());
    return CallbackReturn::ERROR;
  }

  m_current_pose = getEndEffectorPose();
  prev_pos = m_current_pose.pose.position.z;
  m_starting_position = m_current_pose.pose.position;
  m_grid_position = m_starting_position;

  m_ft_sensor_wrench(0) = 0.0;
  m_ft_sensor_wrench(1) = 0.0;
  m_ft_sensor_wrench(2) = 0.0;

  m_target_wrench(0) = 0.0;
  m_target_wrench(1) = 0.0;
  m_target_wrench(2) = 0.0;

  initial_time = get_node()->now();


  m_current_pose = getEndEffectorPose();
  prev_pos = m_current_pose.pose.position.z;
  prec_time = get_node()->now();

  m_phase = 1;
  m_palpation_number = 0;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  m_joint_state_pos_handles.clear();
  m_joint_state_vel_handles.clear();
  this->release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EndEffectorControl::update(const rclcpp::Time & time,
                                                             const rclcpp::Duration & period)
{
  // Control for the palpation
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "Phase: " << m_phase);
  // Print the phase every half a second
  if (time.nanoseconds() % 500000000 == 0)
  {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Phase: " << m_phase);
  }

  // Get the end effector pose
  m_current_pose = getEndEffectorPose();

  m_target_pose.pose.orientation.x = 1;
  m_target_pose.pose.orientation.y = 0;
  m_target_pose.pose.orientation.z = 0;
  m_target_pose.pose.orientation.w = 0;
  // m_current_pose.pose.position.x = m_starting_position.x;
  // m_current_pose.pose.position.y = m_starting_position.y;

  // The controller will palpate the tissue in a square of 0.12x0.12 m starting from the bottom left corner,
  // every palpation is done with a distance of 0.03 m from the previous one. The palpation is done in the z direction
  // and will least 3 seconds. The palpation is done with with a sinusoidal movement of 2hz, a bias of -0.015 m and an amplitude of 0.008 m.

  // Switch case will be used to divide the process in 4 phases:
  // 1. Move the end effector to the position of the palpation
  // 2. Move the end effector in order to touch the surface of the tissue
  // 3. Move the end effector in order to palpate the tCL_issue
  // 4. Move the end effector in order to go back to the initial high

  switch (m_phase)
  {
    case 1:
      gridPosition();
      break;
    case 2:
      surfaceApproach();
      break;
    case 3:
      tissuePalpation(time);
      break;
    case 4:
      startingHigh();
      break;
    default:
      break;
  }

  // Publish the data
  publishDataEE(time);

  return controller_interface::return_type::OK;
}

void EndEffectorControl::gridPosition()
{
  // The end effector will move to the position of the palpation
  m_target_pose.pose.position.x = m_grid_position.x;
  m_target_pose.pose.position.y = m_grid_position.y;
  m_target_pose.pose.position.z = m_starting_position.z;

  m_target_pose.header.stamp = get_node()->now();
  m_target_pose.header.frame_id = m_robot_base_link;

  m_pose_publisher->publish(m_target_pose);

  // If the end effector is in the position of the palpation the phase is finished
  if (abs(m_current_pose.pose.position.x - m_grid_position.x) < 0.005 &&
      abs(m_current_pose.pose.position.y - m_grid_position.y) < 0.005)
  {
    m_phase = 2;
    m_prev_force = 0.0;
    std::cout << "Phase 2" << std::endl;
  }
}

void EndEffectorControl::surfaceApproach()
{
  // If the detected force in the z direction is greater than 10 N the phase is finished
  if (m_ft_sensor_wrench(2) < -4.0)
  {
    std::cout << "Phase 3" << std::endl;
    m_phase = 3;
    m_surface = m_grid_position.z;
    m_target_pose.pose.position.x = m_grid_position.x;
    m_target_pose.pose.position.y = m_grid_position.y;
    m_target_pose.pose.position.z = m_grid_position.z;
    initial_time = get_node()->now();
  }
  // The end effector will move in the z direction until it touches the surface of the tissue
  else
  {
    m_target_pose.pose.position.x = m_grid_position.x;
    m_target_pose.pose.position.y = m_grid_position.y;
    m_grid_position.z -= 0.01 / 500;
    m_target_pose.pose.position.z = m_grid_position.z;
    m_prev_force = m_ft_sensor_wrench(2);
    initial_time = get_node()->now();
  }

  m_target_pose.header.stamp = get_node()->now();
  m_target_pose.header.frame_id = m_robot_base_link;

  m_pose_publisher->publish(m_target_pose);
}

void EndEffectorControl::tissuePalpation(const rclcpp::Time & time)
{
  m_target_pose.pose.position.x = m_grid_position.x;
  m_target_pose.pose.position.y = m_grid_position.y;
  m_target_pose.pose.position.z =
    m_grid_position.z -
    0.003 * sin(2 * M_PI * (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9) * 2);//- 0.001 * sin(2 * M_PI * (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9) * 4);
    
  // if (msgs_queue.size() < 15)
  // {
  //   msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
  //             m_target_pose.pose.position.z, cartVel(2), 0};
  //   msgs_queue.push(msg);
  // }
  // else
  // {
  //   msgs_queue.front().data[5] = m_ft_sensor_wrench(2);
  //   m_data_publisher->publish(msgs_queue.front());

  //   msgs_queue.pop();
  //   msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
  //             m_target_pose.pose.position.z, cartVel(2), 0};
  //   msgs_queue.push(msg);
  // } * sin(2 * M_PI * (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9) * 2);//- 0.001 * sin(2 * M_PI * (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9) * 4);
  // prova di carico
  // std::cout << "x: " << m_target_pose.pose.position.x << std::endl;
  // std::cout << "time: " << time.nanoseconds() * 1e-9 - initial_time.nanoseconds()*1e-9<< std::endl;
  // z position is a sine wave with frequency 10 Hz and amplitude 0.02 m
  //m_target_pose.pose.position.z = m_surface - 0.02 - 0.005 * sin(2 * M_PI * (time.nanoseconds() * 1e-9) * 2.5);

  m_target_pose.header.stamp = get_node()->now();
  m_target_pose.header.frame_id = m_robot_base_link;

  m_pose_publisher->publish(m_target_pose);

  // If the time is greater than 5 seconds the phase is finished
  if (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9 > 20)
  {
    m_grid_position.z = m_grid_position.z -
    0.0015 * sin(2 * M_PI * (time.nanoseconds() * 1e-9 - initial_time.nanoseconds() * 1e-9) * 2);
    std::cout << "Phase 4" << std::endl;
    m_phase = 4;
    m_palpation_number++;
    while (!msgs_queue.empty())
    {
      msgs_queue.pop();
    }
  }
}

void EndEffectorControl::startingHigh()
{
  if (m_current_pose.pose.position.z < m_starting_position.z)
  {
    m_grid_position.z += 0.01 / 500;
  }
  else 
  {
    m_grid_position.z = m_starting_position.z;
  }

  m_target_pose.pose.position.x = m_grid_position.x;
  m_target_pose.pose.position.y = m_grid_position.y;
  m_target_pose.pose.position.z = m_grid_position.z;

  m_target_pose.pose.orientation.x = 1;
  m_target_pose.pose.orientation.y = 0;
  m_target_pose.pose.orientation.z = 0;
  m_target_pose.pose.orientation.w = 0;
  m_target_pose.header.stamp = get_node()->now();
  m_target_pose.header.frame_id = m_robot_base_link;

  m_pose_publisher->publish(m_target_pose);

  // If the end effector is the starting high the phase is finished
  // if (abs(m_current_pose.pose.position.z - m_starting_position.z) < 0.005)
  // {
  //   m_phase = 4;
  //   newStartingPosition();
  //   m_grid_position.z = m_starting_position.z;
  // }
}

void EndEffectorControl::newStartingPosition()
{
  m_grid_position.x = m_starting_position.x + 0.01 * (int)(m_palpation_number / 11);
  m_grid_position.y = m_starting_position.y + 0.01 * (m_palpation_number % 11);
}

controller_interface::InterfaceConfiguration EndEffectorControl::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

void EndEffectorControl::publishDataEE(const rclcpp::Time & time)
{
  // Publish state
  // time, current position, target position, velocity, force
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
              m_target_pose.pose.position.z, cartVel(2), m_ft_sensor_wrench(2)};
  m_data_publisher->publish(msg);

  // Publish state
  // std_msgs::msg::Float64MultiArray msg;
  // msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
  //             m_target_pose.pose.position.z, cartVel(2), m_ft_sensor_wrench(2)};
  // std_msgs::msg::Float64MultiArray msg;
  // if (msgs_queue.size() < 15)
  // {
  //   msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
  //             m_target_pose.pose.position.z, cartVel(2), 0};
  //   msgs_queue.push(msg);
  // }
  // else
  // {
  //   msgs_queue.front().data[5] = m_ft_sensor_wrench(2);
  //   m_data_publisher->publish(msgs_queue.front());

  //   msgs_queue.pop();
  //   msg.data = {(time.nanoseconds() * 1e-9), m_current_pose.pose.position.z,
  //             m_target_pose.pose.position.z, cartVel(2), 0};
  //   msgs_queue.push(msg);
  // }
}

controller_interface::InterfaceConfiguration EndEffectorControl::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * 2);
  for (const auto & type : m_state_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_init()
{
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EndEffectorControl::on_configure(const rclcpp_lifecycle::State & previous_state)
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
    const std::string error =
      ""
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

  m_data_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string("/data_control"), 10);

  m_estimator_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string("/data_estimation"), 10);

  // m_elasticity_publisher = get_node()->create_publisher<std_msgs::msg::Float64>(
  //     std::string("/position_desired"), 10);

  // m_position_publisher = get_node()->create_publisher<std_msgs::msg::Float64>(
  //     std::string("/force_z"), 10);

  // m_force_publisher = get_node()->create_publisher<std_msgs::msg::Float64>(
  //     std::string("/surface"), 10);

  // m_time_publisher = get_node()->create_publisher<std_msgs::msg::Float64>(
  //     std::string("/time"), 10);
  m_state_interface_types.push_back("position");
  m_state_interface_types.push_back("velocity");

  // Subscriber
  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10,
    std::bind(&EndEffectorControl::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
      std::bind(&EndEffectorControl::ftSensorWrenchCallback, this, std::placeholders::_1));

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverVel_recursive(m_robot_chain));
  m_current_pose = getEndEffectorPose();
  Eigen::Quaterniond current_quat(
    m_current_pose.pose.orientation.w, m_current_pose.pose.orientation.x,
    m_current_pose.pose.orientation.y, m_current_pose.pose.orientation.z);
  Eigen::AngleAxisd current_aa(current_quat);
  m_sinusoidal_force.wrench.force.x = 0.0;
  m_sinusoidal_force.wrench.force.y = 0.0;
  m_sinusoidal_force.wrench.force.z = 15.0;
  m_sinusoidal_force.wrench.torque.x = 0.0;
  m_sinusoidal_force.wrench.torque.y = 0.0;
  m_sinusoidal_force.wrench.torque.z = 0.0;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

geometry_msgs::msg::PoseStamped EndEffectorControl::getEndEffectorPose()
{
  KDL::JntArray positions(m_joint_state_pos_handles.size());
  KDL::JntArray velocities(m_joint_state_pos_handles.size());
  for (size_t i = 0; i < m_joint_state_pos_handles.size(); ++i)
  {
    positions(i) = m_joint_state_pos_handles[i].get().get_value();
    velocities(i) = m_joint_state_vel_handles[i].get().get_value();
  }

  KDL::JntArrayVel joint_data(positions, velocities);
  KDL::FrameVel tmp;
  m_fk_solver->JntToCart(joint_data, tmp);

  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = tmp.p.p.x();
  current.pose.position.y = tmp.p.p.y();
  current.pose.position.z = tmp.p.p.z();
  tmp.M.R.GetQuaternion(current.pose.orientation.x, current.pose.orientation.y,
                        current.pose.orientation.z, current.pose.orientation.w);
  
  cartVel(0) = tmp.p.v.x();
  cartVel(1) = tmp.p.v.y();
  cartVel(2) = tmp.p.v.z();

  return current;
}

// geometry_msgs::msg::PoseStamped EndEffectorControl::getEndEffectorVel()
// {
//   KDL::JntArray velocities(m_joint_state_pos_handles.size());
//   for (size_t i = 0; i < m_joint_state_pos_handles.size(); ++i)
//   {
//     velocities(i) = m_joint_state_vel_handles[i].get().get_value();
//   }
//   geometry_msgs::msg::PoseStamped current;

//   RCLCPP_INFO_STREAM(get_node()->get_logger(), "vel arrr: " << velocities.data);
//   return current;
// }

void EndEffectorControl::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;

  m_ft_sensor_wrench(0) = tmp[0];
  m_ft_sensor_wrench(1) = tmp[1];
  m_ft_sensor_wrench(2) = tmp[2];
}

void EndEffectorControl::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;

  m_target_wrench(0) = -tmp[0];
  m_target_wrench(1) = -tmp[1];
  m_target_wrench(2) = -tmp[2];
}

}  // namespace end_effector_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(end_effector_controller::EndEffectorControl,
                       controller_interface::ControllerInterface)
