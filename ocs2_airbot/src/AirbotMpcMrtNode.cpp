/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_airbot/MobileManipulatorDummyVisualization.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ros/init.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

using namespace ocs2;
using namespace mobile_manipulator;

/**
 * This function implements the evaluation of the MPC policy
 * @param currentObservation : current system observation {time, state, input} to compute the input for. (input can be left empty)
 * @param mpcMrtInterface : interface used for communication with the MPC optimization (running in a different thread)
 * @return system input u(t)
 */
ocs2::vector_t mpcTrackingController(
  const ocs2::SystemObservation & currentObservation, ocs2::MPC_MRT_Interface & mpcMrtInterface);

int main(int argc, char ** argv)
{
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;

  // Robot interface
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // Visualization
  mobile_manipulator::MobileManipulatorDummyVisualization dummyVisualization(nodeHandle, interface);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(
    interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
    interface.getOptimalControlProblem(), interface.getInitializer());
  // ROS ReferenceManager
  auto rosReferenceManagerPtr =
    std::make_shared<ocs2::RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  auto observationPublisher =
    nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Create the MPC MRT Interface
  ocs2::MPC_MRT_Interface mpcMrtInterface(mpc);
  mpcMrtInterface.initRollout(&interface.getRollout());

  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
  initObservation.time = 0.0;

  // initial command
  vector_t initTarget(7);
  initTarget.head(3) << 0.3, 0, 0.3;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories(
    {initObservation.time}, {initTarget}, {zeroInput});

  // Set the first observation and command and wait for optimization to finish
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  mpcMrtInterface.setCurrentObservation(initObservation);
  mpcMrtInterface.getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  while (!mpcMrtInterface.initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mpcMrtInterface.advanceMpc();
    ros::WallRate(interface.mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  // airbot interface
  sensor_msgs::JointState joint_states;
  joint_states.position.resize(6);
  auto subscriber_target_joint_vel = nodeHandle.subscribe<sensor_msgs::JointState>(
    "/airbot_play/joint_states", 1, {[&joint_states](const sensor_msgs::JointState::ConstPtr & msg) {
      joint_states = *msg;
    }});
  auto joint_v_piblisher = nodeHandle.advertise<sensor_msgs::JointState>("/airbot_play/set_target_joint_v", 10);

  /*
   * Launch the computation of the MPC in a separate thread.
   * This thread will be triggered at a given frequency and execute an optimization based on the latest available observation.
   */
  std::atomic_bool mpcRunning{true};
  auto mpcThread = std::thread([&]() {
    while (mpcRunning) {
      try {
        ocs2::executeAndSleep(
          [&]() { mpcMrtInterface.advanceMpc(); }, interface.mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception & e) {
        mpcRunning = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  ocs2::setThreadPriority(interface.ddpSettings().threadPriority_, mpcThread);
  /*
   * Main control loop.
   */
  ocs2::SystemObservation currentObservation = initObservation;

  while (mpcRunning && ros::ok()) {
    ocs2::executeAndSleep(  // timed execution of the control loop
      [&]() {
        // ROS_INFO_STREAM("### Current time " << currentObservation.time);
        /*
           * State estimation would go here to fill "currentObservation".
           * In this example we receive the measurement directly after forward integration at the end of the loop.
           */
        // const auto dt = 1.0 / interface.mpcSettings().mrtDesiredFrequency_;
        // currentObservation.time += dt;
        for(uint i = 0; i < currentObservation.state.size(); i++){
          currentObservation.state[i] = joint_states.position[i];
        }

        // Evaluate the control input
        const auto systemInput = mpcTrackingController(currentObservation, mpcMrtInterface);
        // std::cout << "aa" <<systemInput.transpose() << std::endl;
        /*
           * Sending the commands to the actuators would go here.
           * In this example, we instead do a forward simulation + visualization.
           * Simulation is done with the rollout functionality of the mpcMrtInterface, but this can be replaced by any other simulation.
           */
        currentObservation.input = systemInput;
        sensor_msgs::JointState msg;
        msg.velocity.resize(6);
        for (uint i = 0; i < msg.velocity.size(); i++) {
          msg.velocity[i] = systemInput[i];
        }
        joint_v_piblisher.publish(msg);

        // Forward simulation
        // const auto dt = 1.0 / interface.mpcSettings().mrtDesiredFrequency_;
        // ocs2::SystemObservation nextObservation;
        // nextObservation.time = currentObservation.time + dt;
        // mpcMrtInterface.rolloutPolicy(
        //   currentObservation.time, currentObservation.state, dt, nextObservation.state,
        //   nextObservation.input, nextObservation.mode);

        // "state estimation"
        // currentObservation = nextObservation;

        // Visualization
        dummyVisualization.update(
          currentObservation, mpcMrtInterface.getPolicy(), mpcMrtInterface.getCommand());

        // Publish the observation. Only needed for the command interface
        observationPublisher.publish(
          ocs2::ros_msg_conversions::createObservationMsg(currentObservation));

        ros::spinOnce();
      },
      interface.mpcSettings().mrtDesiredFrequency_);
  }

  // Shut down the MPC thread.
  mpcRunning = false;
  if (mpcThread.joinable()) {
    mpcThread.join();
  }

  // Successful exit
  return 0;
}

ocs2::vector_t mpcTrackingController(
  const ocs2::SystemObservation & currentObservation, ocs2::MPC_MRT_Interface & mpcMrtInterface)
{
  // Update the current state of the system
  mpcMrtInterface.setCurrentObservation(currentObservation);

  // Load the latest MPC policy
  bool policyUpdated = mpcMrtInterface.updatePolicy();
  if (policyUpdated) {
    // ROS_INFO_STREAM("<<< New MPC policy received at " << currentObservation.time);
  }

  // Evaluate the current policy
  ocs2::vector_t optimizedState;  // Evaluation of the optimized state trajectory.
  ocs2::vector_t optimizedInput;  // Evaluation of the optimized input trajectory.
  size_t plannedMode;             // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface.evaluatePolicy(
    currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);

  return optimizedInput;
}