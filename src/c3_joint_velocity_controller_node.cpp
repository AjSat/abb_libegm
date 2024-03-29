/***********************************************************************************************************************
 *
 * Copyright (c) 2018, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 * Modified 2019, Wilm Decre, KU Leuven
 */

#include <ros/ros.h>
#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>

int main(int argc, char** argv)
{
  //----------------------------------------------------------
  // Preparations
  //----------------------------------------------------------
  // Initialize the node.
  ros::init(argc, argv, "joint_velocity_controller_node");
  ros::NodeHandle node_handle;

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service;
  boost::thread_group thread_group;

  // Create EGM configurations.
  abb::egm::BaseConfiguration configuration;
  configuration.use_velocity_outputs = true;
  configuration.axes=abb::egm::Seven;

  // Create an EGM interface:
  // * Sets up an EGM server (that the robot controller's EGM client can connect to).
  // * Provides APIs to the user (for setting motion references, that are sent in reply to the EGM client's request).
  //
  // Note: It is important to set the correct port number here,
  //       as well as configuring the settings for the EGM client in thre robot controller.
  //       If using the included RobotStudio Pack&Go file, then port 6511 = ROB_1, 6512 = ROB_2, etc.
  
  // YuMi is configured as: port 6511 = ROB_L; port 6512 = ROB_R
  abb::egm::EGMControllerInterface egm_interface(io_service, 6511, configuration);

  if(!egm_interface.isInitialized())
  {
    ROS_ERROR("EGM interface failed to initialize (e.g. due to port already bound)");
    return 0;
  }

  // Spin up a thread to run the io_service.
  thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  //----------------------------------------------------------
  // Execute a joint velocity controller loop.
  //
  // Note 1: The EGM communication session is started by the
  //         EGMRunJoint RAPID instruction.
  //
  // Note 2: To get pure velocity control, then the EGM client
  //         (in the robot controller) need its position
  //         correction gain to be set to 0. This is done with
  //         the EGMRunJoint RAPID instruction.
  //----------------------------------------------------------
  ROS_INFO("========== Joint velocity controller (open-loop) sample ==========");
  bool wait = true;
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::Joints initial_velocity;
  abb::egm::wrapper::Joints position;
  const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActJoint RAPID instruction).
  int sequence_number = 0;    // [-] (sequence number of a received EGM message).
  double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).

  abb::egm::wrapper::Output output;
  double reference = 0.0;  // [degrees/s].
  double amplitude = 10.0; // [degrees/s].
  double frequency = 0.25; // [Hz].

  ROS_INFO("1: Wait for an EGM communication session to start...");
  while(ros::ok() && wait)
  {
    if(egm_interface.isConnected())
    {
      if(egm_interface.getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        ROS_WARN("RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface.getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }

    ros::Duration(0.5).sleep();
  }

  while(ros::ok())
  {
    // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
    if(egm_interface.waitForMessage(500))
    {
      // Read the message received from the EGM client.
      egm_interface.read(&input);
      sequence_number = input.header().sequence_number();

      if(sequence_number == 0)
      {
        // Reset all references, if it is the first message.
        output.Clear();
        initial_velocity.CopyFrom(input.feedback().robot().joints().velocity());
        output.mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(initial_velocity);
      }
      else
      {
        time = sequence_number/((double) egm_rate);

        // Compute joint velocity reference.
        reference = amplitude*std::sin(2.0*M_PI*frequency*time);

        if(output.mutable_robot()->mutable_joints()->mutable_velocity()->values_size() > 3)
        {
          // Set references for joints
          //output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(0, reference);
          //output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(1, reference);
          //output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(2, reference);
	    output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(5, reference);
	    output.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(6, reference);
        }

        if(sequence_number%egm_rate == 0)
        {
	  // Print reference position
          ROS_INFO_STREAM("Reference = " << reference << " [degrees/s]");
	  // Get current position
	  position.CopyFrom(input.feedback().robot().joints().position());
	  // Print current position
  	  ROS_INFO_STREAM("Position = " << position.values(0) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(1) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(2) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(3) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(4) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(5) << " [degrees]");
  	  ROS_INFO_STREAM("Position = " << position.values(6) << " [degrees]");
        }
      }

      // Write references back to the EGM client.
      egm_interface.write(output);
    }
  }

  // Perform a clean shutdown.
  io_service.stop();
  thread_group.join_all();

  return 0;
}
