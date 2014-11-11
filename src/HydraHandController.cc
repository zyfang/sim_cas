/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Andrei Haidu,
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

#include "HydraHandController.hh"
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/LogRecord.hh>
#include <gazebo/transport/transport.hh>

#define PI 3.14159265359

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(HydraHandController)

//////////////////////////////////////////////////
HydraHandController::HydraHandController() : offsetPos(0.8, 0, 0.8),
offsetQuat(-PI/2 + 0.3, 0, -PI/2)
{
}

//////////////////////////////////////////////////
HydraHandController::~HydraHandController()
{
}

//////////////////////////////////////////////////
void HydraHandController::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Get the world name.
	this->world = _parent;

	// Get the model to be controlled by hydra
	this->handModel = this->world->GetModel("Hand");

	// disable gravity on the hand model
	this->handModel->SetGravityMode(false);

	// get hand joint controller
	this->jointController = this->handModel->GetJointController();

	// Initialize the transport node
	this->gznode = transport::NodePtr(new transport::Node());

	this->gznode->Init(this->world->GetName());

	// Subscribe to hydra topic
	this->hydraSub = this->gznode->Subscribe("~/hydra",
			&HydraHandController::OnHydra, this);

	// Subscribe to the given contact topic
	this->thumbContactSub = this->gznode->Subscribe(
			"~/thumb_contact", &HydraHandController::OnThumbContact, this);

	// Subscribe to the given contact topic
	this->foreContactSub = this->gznode->Subscribe(
			"~/fore_finger_contact", &HydraHandController::OnForeFingerContact, this);

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&HydraHandController::OnUpdate, this));

	// set the initial desired position
	this->desiredPosition = this->handModel->GetWorldPose().pos;

	// set the initial desired orientation
	this->desiredQuat = this->handModel->GetWorldPose().rot;

	// init hand movement state flag
	this->pauseHand = true;

	// init pause button state flag
	this->pauseButtonPressed = false;

	// initialize gripper state flags
	this->idleGripper = true;
	this->jointAttached = false;
	this->closingGripper = false;
	this->foreFingerInContact = false;

	// set up the PID values
	double const _control_P = 100;
	double const _control_I = 0;
	double const _control_D = 25;

	// add pid controllers for pos
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));

	// add pid controllers for rot
	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));

	// set the initial finger positions
	HydraHandController::InitFingerPos();

	std::cout << "******** HYDRA HAND CONTROLLER LOADED *********" << std::endl;
}

/////////////////////////////////////////////////
void HydraHandController::OnUpdate()
{
	// compute step time
	common::Time step_time = this->world->GetSimTime() - this->prevSimTime;
	this->prevSimTime = this->world->GetSimTime();

	// brief apply forces in order to control the hand Pose
	HydraHandController::HandPoseControl(step_time);
}

/////////////////////////////////////////////////
void HydraHandController::OnHydra(ConstHydraPtr &_msg)
{

	// check pause button status
	if (_msg->right().button_center())
	{
		// button pressed
		this->pauseButtonPressed = true;
	}

	// if button released toggle pause hand button
	if(this->pauseButtonPressed && !_msg->right().button_center())
	{
		// set button to released, and toggle pause hand
		this->pauseButtonPressed = false;
		this->pauseHand = !this->pauseHand;
	}

	// if hand is paused return from the callback
	if (this->pauseHand)
		return;

	// update the desired pose of the hand
	HydraHandController::UpdateHandPose(_msg);

	// brief control fingers
	HydraHandController::FingersControl(_msg->right().joy_x(), _msg->right().joy_y());
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::OnThumbContact(ConstContactsPtr &_msg)
{
	// if gripper is not closing/opening
	if (this->idleGripper)
	{
		return;
	}
	// if the thumb is in contact, the gripper is closing and no joint is attached then attach the joint
	else if (this->closingGripper && _msg->contact_size() > 0 && !this->jointAttached && this->foreFingerInContact)
	{
		boost::char_separator<char> sep("::");

		boost::tokenizer< boost::char_separator<char> > tok(_msg->contact(0).collision1(), sep);
		boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();

		physics::ModelPtr attachModel = this->world->GetModel(*beg);
		//std::cout << "attach Model: " << attachModel->GetName().c_str() << std::endl;
		beg++;
		physics::LinkPtr attachLink = attachModel->GetLink(*beg);
		//std::cout << "attach Link: " << attachLink->GetName().c_str() << std::endl;

		HydraHandController::AttachJoint(attachLink);
	}
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::OnForeFingerContact(ConstContactsPtr &_msg)
{
	if(_msg->contact_size()>0)
	{
		this->foreFingerInContact = true;
	}
	else
	{
		this->foreFingerInContact = false;
	}
}

//////////////////////////////////////////////////
void HydraHandController::HandPoseControl(const common::Time _step_time)
{
	// get current pose of the hand
	const math::Pose curr_pose = this->handModel->GetWorldPose();

	// the translation and rotation values to be applied on the hand
	math::Vector3 pos_effort, rot_effort;

	// PID values for hand control
	// computing PID values from the current position, for the x, y, z axis
	pos_effort.x = this->controlPIDs[0].Update(curr_pose.pos.x - this->desiredPosition.x, _step_time);
	pos_effort.y = this->controlPIDs[1].Update(curr_pose.pos.y - this->desiredPosition.y, _step_time);
	pos_effort.z = this->controlPIDs[2].Update(curr_pose.pos.z - this->desiredPosition.z, _step_time);

	// compute rotation velocity values from the current quaternion
	rot_effort = HydraHandController::ReturnRotVelocity(curr_pose.rot);

	// apply the computed lin/rot forces/velocities to the model
	this->handModel->GetLink("palm_link")->SetForce(pos_effort);
	this->handModel->SetAngularVel(rot_effort);
}

//////////////////////////////////////////////////
void HydraHandController::InitFingerPos()
{
	// Limits example
	// thumb            lower="0"              upper="1.570796"
	// thumb base       lower="-0.261799388"   upper="0.261799388"
	// thumb proximal   lower="-0.087266463"   upper="1.134464014"
	// thumb middle     lower="0.087266463"    upper="1.134464014"
	// thumb distal     lower="0.087266463"    upper="1.134464014"

	this->jointController->SetPositionTarget("Hand::thumb_joint", 1.57);

	this->jointController->SetPositionTarget("Hand::thumb_base_joint", 0);
	this->jointController->SetPositionTarget("Hand::thumb_proximal_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::thumb_middle_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::thumb_distal_joint", 0.1);

	this->jointController->SetPositionTarget("Hand::fore_finger_base_joint", 0);
	this->jointController->SetPositionTarget("Hand::fore_finger_proximal_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::fore_finger_middle_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::fore_finger_distal_joint", 0.1);

	this->jointController->SetPositionTarget("Hand::middle_finger_base_joint", 0);
	this->jointController->SetPositionTarget("Hand::middle_finger_proximal_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::middle_finger_middle_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::middle_finger_distal_joint", 0.1);

	this->jointController->SetPositionTarget("Hand::ring_finger_base_joint", 0);
	this->jointController->SetPositionTarget("Hand::ring_finger_proximal_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::ring_finger_middle_joint", 0.1);
	this->jointController->SetPositionTarget("Hand::ring_finger_distal_joint", 0.1);
}

//////////////////////////////////////////////////
void HydraHandController::FingersControl(const double _joy_x, const double _joy_y)
{
	// opposite thumb movement
	if (_joy_y != 0)
	{
		// current target pos
		const double curr_pos = this->jointController->GetPositions()["Hand::thumb_joint"];

		const double target_pos = curr_pos - (_joy_y / 500);

		// move thumb if it's between the limits
		if (target_pos < 1.57 && target_pos > 0.01)
		{
			this->jointController->SetPositionTarget("Hand::thumb_joint", target_pos);
		}
	}

//	if (_joy_x != 0)
//	{
//		std::map<std::string, physics::JointPtr> joints_M = this->jointController->GetJoints();
//
//		std::cout << joints_M.size() << std::endl;
//
//		for(std::map<std::string, physics::JointPtr>::const_iterator joint_iter = joints_M.begin();
//				joint_iter != joints_M.end(); ++joint_iter)
//		{
//			std::cout << joint_iter->first << joint_iter->second->GetName() << std::endl;
//		}
//
//	}

	//		std::map<std::string, double> positions_M = this->jointController->GetPositions();
	//
	//		std::cout << positions_M.size() << std::endl;
	//
	//		for(std::map<std::string, double>::const_iterator joint_iter = positions_M.begin();
	//				joint_iter != positions_M.end(); ++joint_iter)
	//		{
	//			std::cout << joint_iter->first << joint_iter->second << std::endl;
	//		}
}

//////////////////////////////////////////////////
math::Vector3 HydraHandController::ReturnRotVelocity(const math::Quaternion _curr_quat)
{
    math::Vector3 rot_velocity;

    const math::Quaternion quatern_diff = (this->desiredQuat - _curr_quat) * 3.0;

    const math::Quaternion vel_q = quatern_diff * _curr_quat.GetInverse();

    rot_velocity.x = vel_q.x * 8;
    rot_velocity.y = vel_q.y * 8;
    rot_velocity.z = vel_q.z * 8;

    return rot_velocity;
}

//////////////////////////////////////////////////
void HydraHandController::UpdateHandPose(ConstHydraPtr &_msg)
{
	// set the desired orientation from the received msg
	const math::Quaternion q_msg = math::Quaternion(
			-_msg->right().pose().orientation().z(),
			_msg->right().pose().orientation().y(),
			-_msg->right().pose().orientation().x(),
			_msg->right().pose().orientation().w());

	// apply offset to the orientation
	this->desiredQuat = q_msg * this->offsetQuat;

	// set the desired position
	this->desiredPosition.x = _msg->right().pose().position().x() * 2.5 + this->offsetPos.x;
	this->desiredPosition.y = _msg->right().pose().position().y() * 2.5 + this->offsetPos.y;
	this->desiredPosition.z = _msg->right().pose().position().z() * 2.5 + this->offsetPos.z;
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::AttachJoint(const physics::LinkPtr attachLink)
{
	const physics::LinkPtr palm_link = this->handModel->GetLink("palm_link");

	std::cout << "Creating joint between " << palm_link->GetName().c_str()
			<< " and " << attachLink->GetName().c_str() << std::endl;

	// creating joint
	this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint("revolute", this->handModel);

	// attaching and setting the joint to fixed
	this->fixedJoint->Load(palm_link, attachLink, math::Pose());
    this->fixedJoint->Init();
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);

    // set attached flag to true
	this->jointAttached = true;
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::DetachJoint()
{
	std::cout << "Detaching Fixed Joint! " <<  std::endl;

	// removing and detaching joint
	this->fixedJoint->Reset();
	this->fixedJoint->Detach();
	this->fixedJoint->Fini();

	// set attached flag to false
	this->jointAttached = false;
}

