/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andrei Haidu,
 *  Institute for Artificial Intelligence, Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef HYDRA_PR2_R_GRIPPER_HH
#define HYDRA_PR2_R_GRIPPER_HH

#include "gazebo/physics/physics.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/util/LogRecord.hh"

namespace sim_games
{   

/// \brief HydraPR2RGripper class
class HydraPR2RGripper : public gazebo::ModelPlugin
{
    /// \brief Constructor
    public: HydraPR2RGripper();

    /// \brief Destructor
    public: virtual ~HydraPR2RGripper();

    /// \brief Load plugin
    protected: virtual void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update callback function called on every world update event
    protected: virtual void OnUpdate();

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief apply forces in order to control the hand Pose
    private: void GripperPoseControl(const gazebo::common::Time _step_time);

    /// \brief Update the desired pose of the robot gripper
    private: void UpdateGripperPose(ConstHydraPtr& _msg);

    /// \brief Control the gripper positions / open-close force
    private: void GripperControl(const double _joy_x);

    /// \brief Return the required rotational velocity
    private: gazebo::math::Vector3 ReturnRotVelocity(gazebo::math::Quaternion _curr_quat);

    /// \brief Callback right gripper tip contact sensor
    private: void OnRGripperContact(ConstContactsPtr &_msg);

    /// \brief Callback left gripper tip contact sensor
    private: void OnLGripperContact(ConstContactsPtr &_msg);

    /// \brief attach joint with the end effector
    private: void AttachJoint();

    /// \brief detach joint from the end effector
    private: void DetachJoint();

    /// \brief Toggle between logging the world or not
    private: void ToggleLogging(const bool _btn);

    /// \brief World pointer
    private: gazebo::physics::WorldPtr world;

    /// \brief model of the robot hand
    private: gazebo::physics::ModelPtr gripperModel;

    /// \brief world update event
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: gazebo::transport::NodePtr gznode;

    /// \brief Subscribe to hydra topic.
    private: gazebo::transport::SubscriberPtr hydraSub;

    /// \brief Right gripper sensor subscriber
    private: gazebo::transport::SubscriberPtr rGripperSub;

    /// \brief Left gripper sensor subscriber
    private: gazebo::transport::SubscriberPtr lGripperSub;    
        
    /// \brief Custom event msg publisher (e.g. start/stop logging)
    private: gazebo::transport::PublisherPtr customEventPub;

    /// \brief Gripper joints controller for opening closing the gripper
    /// using position control
    private: gazebo::physics::JointControllerPtr jointController;

    /// \brief Right screw joint of the gripper for opening/closing the gripper
    /// using force control
    private: gazebo::physics::JointPtr rScrewJoint;

    /// \brief left screw joint of the gripper
    private: gazebo::physics::JointPtr lScrewJoint;

    /// \brief Positional offset for the gripper
    private: const gazebo::math::Vector3 offsetPos;

    /// \brief Orientation offset for the gripper
    private: const gazebo::math::Quaternion offsetQuat;

    /// \brief Desired position of the hand
    private: gazebo::math::Vector3 desiredPosition;

    /// \brief Desired orientation of the hand
    private: gazebo::math::Quaternion desiredQuat;

    /// \brief Hand joint position PID controllers
    private: std::vector<gazebo::common::PID> posPIDs;

    /// \brief Timestamp of the last cycle for the PID
    private: gazebo::common::Time prevSimTime;

    /// \brief Hand movement state flag
    private: bool disableHydra;

    /// \brief End effector disable button state
    private: bool disableBtnPressed;

    /// \brief Fixate joint attached to the end effector
    private: bool jointAttached;

    /// \brief Right tip is in contact with something
    private: bool rTipInContact;

    /// \brief Right tip is in contact with something
    private: bool lTipInContact;

    /// \brief Gripper in idle state
    private: bool idleGripper;

    /// \brief Gripper in closing state
    private: bool closingGripper;

    /// \brief Joint pointer used to fixate on objects
    private: gazebo::physics::JointPtr fixedJoint;

    /// \brief Model name in contact with the right tip
    private: std::string rContactModelName;

    /// \brief Model name in contact with the left tip
    private: std::string lContactModelName;

    /// \brief Flags for start/stop logging the simulation
    private: bool loggingOn;

    /// \brief Log world button state
    private: bool logBtnPressed;

    /// \brief Attached model
    private: gazebo::physics::ModelPtr attachedModel;

};
}
#endif
