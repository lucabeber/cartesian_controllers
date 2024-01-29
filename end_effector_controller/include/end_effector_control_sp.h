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
/*!\file    motion_control_handle.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/11/06
 *
 */
//-----------------------------------------------------------------------------

#ifndef END_EFFECTOR_CONTROL_H_INCLUDED
#define END_EFFECTOR_CONTROL_H_INCLUDED

#include "cartesian_controller_base/ROS2VersionConfig.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/publisher.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <queue>

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "SystemModelF.hpp"
#include "ForceMeasurementModel.hpp"

#include <end_effector_controller/kalman/ExtendedKalmanFilter.hpp>
#include <end_effector_controller/kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

using namespace KalmanExamples2;

typedef float T;

// Some type shortcuts
typedef Estimation::State<T> State;
typedef Estimation::SystemModel<T> SystemModel;
typedef Estimation::Control<T> Control;

typedef Estimation::VelocityMeasurement<T> VelocityMeasurement;
typedef Estimation::ForceMeasurementModel<T> ForceModel;

namespace end_effector_controller
{

/**
 * @brief Implements a drag-and-drop control handle in RViz
 *
 * This motion control handle can be used to control the end effector of a
 * robotic manipulator in Cartesian space with an interactive marker.
 * A common use case is quick teaching of Cartesian motion.
 * The motion of the marker (both orientation and translation) is published as a
 * geometry_msgs/PoseStamped, which can be followed by motion-based Cartesian
 * controllers, such as the \ref CartesianMotionController or the \ref
 * CartesianComplianceController.
 */
class EndEffectorControl : public controller_interface::ControllerInterface
{
  public:
    EndEffectorControl();
    ~EndEffectorControl();

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
    virtual LifecycleNodeInterface::CallbackReturn on_init() override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
    virtual controller_interface::return_type init(const std::string & controller_name) override;
#endif

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;



  private:
    /**
     * @brief Move visual marker in RViz according to user interaction
     *
     * This function also stores the marker pose internally.
     *
     * @param feedback The message containing the current pose of the marker
     */
    void updateMotionControlCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

    /**
     * @brief Get the current pose of the specified end-effector
     *
     * @return The current end-effector pose with respect to the specified base link
     */
    geometry_msgs::msg::PoseStamped getEndEffectorPose();

    geometry_msgs::msg::PoseStamped getEndEffectorVel();
    /**
     * @brief Proportional controller to set the orientation perpendicular to the surfae
     * 
     * @param currentPos The end effector current pose
     * 
     * @return The new orientation of the end effector
     */
    geometry_msgs::msg::Quaternion setEndEffectorOrientation(geometry_msgs::msg::Quaternion pos);

    void ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
    void targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

    // Handles to the joints
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
      m_joint_state_pos_handles;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
      m_joint_state_vel_handles;

    std::vector<std::string>  m_joint_names;
    std::vector<std::string> m_state_interface_types;

    // Kinematics
    std::string   m_robot_base_link;
    std::string   m_end_effector_link;
    std::string   m_target_frame_topic;
    KDL::Chain    m_robot_chain;
    std::shared_ptr<
      KDL::ChainFkSolverVel_recursive>  m_fk_solver;

    geometry_msgs::msg::PoseStamped  m_current_pose;
    geometry_msgs::msg::WrenchStamped  m_sinusoidal_force;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  m_pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  m_data_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  m_estimator_publisher;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  m_elasticity_publisher;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  m_position_publisher;
    // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  m_force_publisher;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft_sensor_wrench_subscriber;
    Eigen::Vector3d     m_target_wrench;
    Eigen::Vector3d     m_ft_sensor_wrench;
    Eigen::Vector3d     errorOrientation;
    Eigen::Vector3d     cartVel;
    geometry_msgs::msg::Point starting_position;
    uint tr_counter;
    // // Interactive marker
    // std::shared_ptr<
    //   interactive_markers::InteractiveMarkerServer> m_server;

    // visualization_msgs::msg::InteractiveMarker           m_marker; //!< Controller handle for RViz

    public:
    // Simulated (true) system state
    State x;

    // System
    SystemModel sys;

    // Measurement models
    ForceModel fm;

    Kalman::ExtendedKalmanFilter<State> ekf;

    Kalman::Covariance<State> cov;

    // Previos position
    float prev_pos;
    rclcpp::Time initial_time;
    rclcpp::Time prec_time;

    std::queue<std_msgs::msg::Float64MultiArray> msgs_queue;

};

} // cartesian_controller_handles

#endif
