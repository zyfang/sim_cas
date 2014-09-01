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

#include "HydraGameController.hh"
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/util/LogRecord.hh"
#include "gazebo/transport/transport.hh"

#define PI 3.14159265359

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(HydraGameController)

//////////////////////////////////////////////////
HydraGameController::HydraGameController() : offsetPos(0.8, 0, 0.8), offsetQuat(-PI/2,0,-PI/2)
{

}

//////////////////////////////////////////////////
HydraGameController::~HydraGameController()
{

}

//////////////////////////////////////////////////
void HydraGameController::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Get the world name.
	this->world = _parent;

	// Get the model to be controlled by hydra
	this->handModel = this->world->GetModel("hit_hand");

	// get log record instance
	this->logRecorder = util::LogRecord::Instance();

	// TODO add as an sdf param
	// set logging path directory
	this->logRecorder->SetBasePath("Logs");

	// Initialize the transport node
	this->gznode = transport::NodePtr(new transport::Node());
	this->gznode->Init(this->world->GetName());

	// Subscribe to hydra topic
	this->hydraSub = this->gznode->Subscribe("~/hydra",
			&HydraGameController::OnHydra, this);

	// Subscribe to the given contact topic
	this->thumbContactSub = this->gznode->Subscribe(
			"~/thumb_contact", &HydraGameController::OnThumbContact, this);

	// initialize the contact subscriber to the given topic, and callback OnContact
	this->foreContactSub = this->gznode->Subscribe(
			"~/fore_finger_contact", &HydraGameController::OnForeFingerContact, this);

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&HydraGameController::Update, this, _1));

	// set the hand to be paused, and the status of the button
	this->pauseHand = true;
	this->pauseButtonPressed = false;

	// set the log flags
	this->logBtnPressed = false;
	this->logOn = false;

	// initialize flags
	this->idleGripper = true;
	this->jointAttached = false;
	this->closingGripper = false;
	this->foreFingerInContact = false;

	// disable gravity on the hand model
	this->handModel->SetGravityMode(false);

    // Set hand joints
	HydraGameController::SetHandJoints();

    // Set hand joints positions
	HydraGameController::SetJointsPostion();

    // Set up control and finger joint PIDs
	HydraGameController::SetJointsPIDs();

	// set desired position the current hand position (the spawned position)
	this->desiredPosition = this->handModel->GetWorldPose().pos;

	// set the initial value of the rotation
	this->desiredQuat.Set(0.0, 0.0, 0.0, 1.0);

	std::cout << "******** HYDRA CONTROLLER LOADED *********" << std::endl;
}

/////////////////////////////////////////////////
void HydraGameController::Update(const common::UpdateInfo & /*_info*/)
{
	// compute step time
	common::Time step_time = this->world->GetSimTime() - this->prevSimTime;
	this->prevSimTime = this->world->GetSimTime();

	// brief apply forces in order to control the hand Pose
	HydraGameController::HandPoseControl(step_time);

	// brief apply forces in order to control the finger positions
	HydraGameController::FingersControl(step_time);
}

/////////////////////////////////////////////////
void HydraGameController::OnHydra(ConstHydraPtr &_msg)
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
	HydraGameController::UpdateDesiredPose(_msg);

	// open / close gripper depending on the received gestures
	if(_msg->right().joy_x() != 0)
	{
		// set flag that the gripper is closing or opening
		this->idleGripper = false;

		if (_msg->right().joy_x() > 0 && !this->jointAttached)
		{
			// Close gripper
			this->closingGripper = true;
			HydraGameController::CloseGripper();
		}
		else if (_msg->right().joy_x() < 0)
		{
			// Open gripper
			this->closingGripper = false;
			HydraGameController::OpenGripper();
		}
	}
	else
	{
		// gripper is idle
		this->idleGripper = true;
	}


	// check for logging
	if (_msg->right().button_3())
	{
		// button pressed
		this->logBtnPressed = true;
	}

	if(this->logBtnPressed && !_msg->right().button_3())
	{

		if (!this->logOn)
		{
			std::cout << "Starting logging.." << std::endl;

			// start recording with given compression type
			util::LogRecord::Instance()->Start("txt"); // txt, bz2, zlib

			// set flag to true
			this->logOn = true;
		}
		else
		{
			std::cout << "Stop logging.." << std::endl;

			// stop recording
			util::LogRecord::Instance()->Stop();

			// set flag to false
			this->logOn = false;
		}

		// button released
		this->logBtnPressed = false;
	}
}

//////////////////////////////////////////////////
void HydraGameController::UpdateDesiredPose(ConstHydraPtr &_msg)
{
	math::Quaternion q_msg;

	q_msg.Set(- _msg->right().pose().orientation().z(),
			   _msg->right().pose().orientation().y(),
			  - _msg->right().pose().orientation().x(),
			   _msg->right().pose().orientation().w());

	this->desiredQuat = q_msg * this->offsetQuat;

	this->desiredPosition.x = _msg->right().pose().position().x() * 2 + this->offsetPos.x;
	this->desiredPosition.y = _msg->right().pose().position().y() * 2 + this->offsetPos.y;
	this->desiredPosition.z = _msg->right().pose().position().z() * 2 + this->offsetPos.z;

}

//////////////////////////////////////////////////
void HydraGameController::SetHandJoints()
{
    // set up thumb joints
	// thumb origin joint
	this->thumbFingerJoints.push_back(this->handModel->GetJoint("thumb_joint"));
	// thumb finger
    this->thumbFingerJoints.push_back(this->handModel->GetJoint("thumb_base_joint"));
    this->thumbFingerJoints.push_back(this->handModel->GetJoint("thumb_proximal_joint"));
    this->thumbFingerJoints.push_back(this->handModel->GetJoint("thumb_middle_joint"));
    this->thumbFingerJoints.push_back(this->handModel->GetJoint("thumb_distal_joint"));

    // fore finger
    this->foreFingerJoints.push_back(this->handModel->GetJoint("fore_finger_base_joint"));
    this->foreFingerJoints.push_back(this->handModel->GetJoint("fore_finger_proximal_joint"));
    this->foreFingerJoints.push_back(this->handModel->GetJoint("fore_finger_middle_joint"));
    this->foreFingerJoints.push_back(this->handModel->GetJoint("fore_finger_distal_joint"));

    // middle finger
    this->middleFingerJoints.push_back(this->handModel->GetJoint("middle_finger_base_joint"));
    this->middleFingerJoints.push_back(this->handModel->GetJoint("middle_finger_proximal_joint"));
    this->middleFingerJoints.push_back(this->handModel->GetJoint("middle_finger_middle_joint"));
    this->middleFingerJoints.push_back(this->handModel->GetJoint("middle_finger_distal_joint"));

    // ring finger
    this->ringFingerJoints.push_back(this->handModel->GetJoint("ring_finger_base_joint"));
    this->ringFingerJoints.push_back(this->handModel->GetJoint("ring_finger_proximal_joint"));
    this->ringFingerJoints.push_back(this->handModel->GetJoint("ring_finger_middle_joint"));
    this->ringFingerJoints.push_back(this->handModel->GetJoint("ring_finger_distal_joint"));
}

//////////////////////////////////////////////////
void HydraGameController::SetJointsPostion()
{
    //set up the joints initial position
    this->thumbJointsPos.push_back(1.57); //thumb            lower="0"              upper="1.570796"
    this->thumbJointsPos.push_back(0.0);  //thumb base       lower="-0.261799388"   upper="0.261799388"
    this->thumbJointsPos.push_back(0.1);  //thumb proximal   lower="-0.087266463"   upper="1.134464014"
    this->thumbJointsPos.push_back(0.1);  //thumb middle     lower="0.087266463"    upper="1.134464014"
    this->thumbJointsPos.push_back(0.1);  //thumb distal     lower="0.087266463"    upper="1.134464014"

    this->foreJointsPos.push_back(0.0);   //ring base        lower="-0.261799388"   upper="0.261799388"
    this->foreJointsPos.push_back(0.1);   //ring proximal    lower="-0.087266463"   upper="1.134464014"
    this->foreJointsPos.push_back(0.1);   //ring middle      lower="0.087266463"    upper="1.134464014"
    this->foreJointsPos.push_back(0.1);   //ring distal      lower="0.087266463"    upper="1.134464014"

    this->middleJointsPos.push_back(0.0); //middle base      lower="-0.261799388"   upper="0.261799388"
    this->middleJointsPos.push_back(0.1); //middle proximal  lower="-0.087266463"   upper="1.134464014"
    this->middleJointsPos.push_back(0.1); //middle middle    lower="0.087266463"    upper="1.134464014"
    this->middleJointsPos.push_back(0.1); //middle distal    lower="0.087266463"    upper="1.134464014"

    this->ringJointsPos.push_back(0.0);   //fore base        lower="-0.261799388"   upper="0.261799388"
    this->ringJointsPos.push_back(0.1);   //fore proximal    lower="-0.087266463"   upper="1.134464014"
    this->ringJointsPos.push_back(0.1);   //fore middle      lower="0.087266463"    upper="1.134464014"
    this->ringJointsPos.push_back(0.1);   //fore distal      lower="0.087266463"    upper="1.134464014"

}

//////////////////////////////////////////////////
void HydraGameController::SetJointsPIDs()
{
	double const _control_P = 27;
	double const _control_I = 0;
	double const _control_D = 15;
	double const _finger_P = 2;
	double const _finger_I = 0;
	double const _finger_D = 0;

	// set up control PIDs for the 3 axis XYZ
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));
	this->controlPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));

	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));
	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));
	this->rotPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 10, -10));


	// set up thumb joint PIDs
	for (unsigned int i=0; i < this->thumbFingerJoints.size(); i++)
	{
		this->thumbJointPIDs.push_back(common::PID(_finger_P, _finger_I, _finger_D, 10, -10));
	}

	// set up fore finger joint PIDs
	for (unsigned int i=0; i < this->foreFingerJoints.size(); i++)
	{
		this->foreJointPIDs.push_back(common::PID(_finger_P, _finger_I, _finger_D, 10, -10));
	}

	// set up middle finger joint PIDs
	for (unsigned int i=0; i < this->middleFingerJoints.size(); i++)
	{
		this->middleJointPIDs.push_back(common::PID(_finger_P, _finger_I, _finger_D, 10, -10));
	}

	// set up ring finger joint PIDs
	for (unsigned int i=0; i < this->ringFingerJoints.size(); i++)
	{
		this->ringJointPIDs.push_back(common::PID(_finger_P, _finger_I, _finger_D, 10, -10));
	}
}

//////////////////////////////////////////////////
void HydraGameController::HandPoseControl(const common::Time _step_time)
{
	// get current pose of the hand
	const math::Pose curr_pose = this->handModel->GetWorldPose();

	// the translation and rotation values to be applied on the hand
	math::Vector3 pid_lin_velocity, rot_velocity;

	// PID values for hand control
	// computing PID values from the current position, for the x, y, z axis
	pid_lin_velocity.x = this->controlPIDs[0].Update(curr_pose.pos.x - this->desiredPosition.x, _step_time);
	pid_lin_velocity.y = this->controlPIDs[1].Update(curr_pose.pos.y - this->desiredPosition.y, _step_time);
	pid_lin_velocity.z = this->controlPIDs[2].Update(curr_pose.pos.z - this->desiredPosition.z, _step_time);

	// compute rotation velocity values from the current quaternion
	rot_velocity = HydraGameController::ReturnRotVelocity(curr_pose.rot);

	// apply the computed lin/rot forces/velocities to the model
	this->handModel->GetLink("palm_link")->SetForce(pid_lin_velocity);
	this->handModel->SetAngularVel(rot_velocity);
}

//////////////////////////////////////////////////
math::Vector3 HydraGameController::ReturnRotVelocity(const math::Quaternion _curr_quat)
{
    math::Vector3 rot_velocity;
    math::Quaternion quatern_diff, vel_q;

    quatern_diff = (this->desiredQuat - _curr_quat) * 2.0;
    vel_q = quatern_diff * _curr_quat.GetInverse();

    rot_velocity.x = vel_q.x*6;
    rot_velocity.y = vel_q.y*6;
    rot_velocity.z = vel_q.z*6;

    return rot_velocity;
}

//TODO check for limits, or do it similarly to like pr2 gripper,
// apply force directly and the limit will stop it
//////////////////////////////////////////////////
void HydraGameController::FingersControl(const common::Time _step_time)
{
	double _error, _effort;

	// PID values for finger joint control
	for (unsigned int i=0; i < this->thumbFingerJoints.size(); i++)
	{
		// compute the error (difference between the current position and the desired one
		_error = this->thumbFingerJoints[i]->GetAngle(0).Radian() - this->thumbJointsPos[i];

		// compute the effort wit the corresponding PID
		 _effort = this->thumbJointPIDs[i].Update(_error, _step_time);

		// apply effort on the joint
		this->thumbFingerJoints[i]->SetForce(0, _effort);

	}

	for (unsigned int i=0; i < this->foreFingerJoints.size(); i++)
	{
		// compute the error (difference between the current position and the desired one
		_error = this->foreFingerJoints[i]->GetAngle(0).Radian() - this->foreJointsPos[i];

		// compute the effort wit the corresponding PID
		_effort = this->foreJointPIDs[i].Update(_error, _step_time);

		// apply effort on the joint
		this->foreFingerJoints[i]->SetForce(0, _effort);
	}

	for (unsigned int i=0; i < this->middleFingerJoints.size(); i++)
	{
		// compute the error (difference between the current position and the desired one
		_error = this->middleFingerJoints[i]->GetAngle(0).Radian() - this->middleJointsPos[i];

		// compute the effort wit the corresponding PID
		_effort = this->middleJointPIDs[i].Update(_error, _step_time);

		// apply effort on the joint
		this->middleFingerJoints[i]->SetForce(0, _effort);
	}

	for (unsigned int i=0; i < this->ringFingerJoints.size(); i++)
	{
		// compute the error (difference between the current position and the desired one
		_error = this->ringFingerJoints[i]->GetAngle(0).Radian() - this->ringJointsPos[i];

		// compute the effort wit the corresponding PID
		_effort = this->ringJointPIDs[i].Update(_error, _step_time);

		// apply effort on the joint
		this->ringFingerJoints[i]->SetForce(0, _effort);
	}
}

//////////////////////////////////////////////////
void HydraGameController::OpenGripper()
{
	if (this->thumbJointsPos[2] > 0)
	{
		const double _step = 0.0005;
		this->thumbJointsPos[2] 	-= _step;
		this->thumbJointsPos[3] 	-= _step;

		this->foreJointsPos[1] 		-= _step/2;
		this->foreJointsPos[2] 		-= _step/2;

		this->middleJointsPos[1] 	-= _step/2;
		this->middleJointsPos[2] 	-= _step/2;

		this->ringJointsPos[1] 		-= _step/2;
		this->ringJointsPos[2] 		-= _step/2;
	}

	if(this->jointAttached)
	{
		HydraGameController::DetachJoint();
	}
}

//////////////////////////////////////////////////
void HydraGameController::CloseGripper()
{
	if (this->thumbJointsPos[2] < 1)
	{
		const double _step = 0.0005;
		this->thumbJointsPos[2] 	+= _step;
		this->thumbJointsPos[3] 	+= _step;

		this->foreJointsPos[1] 		+= _step/2;
		this->foreJointsPos[2] 		+= _step/2;

		this->middleJointsPos[1] 	+= _step/2;
		this->middleJointsPos[2] 	+= _step/2;

		this->ringJointsPos[1] 		+= _step/2;
		this->ringJointsPos[2] 		+= _step/2;
	}
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraGameController::OnThumbContact(ConstContactsPtr &_msg)
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

		HydraGameController::AttachJoint(attachLink);
	}
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraGameController::OnForeFingerContact(ConstContactsPtr &_msg)
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

//////////////////////////////////////////////////////////////////////////////////////
void HydraGameController::AttachJoint(const physics::LinkPtr attachLink)
{
	physics::LinkPtr palm_link = this->handModel->GetLink("palm_link");

	std::cout << "Creating joint between " << palm_link->GetName().c_str()
			<< " and " << attachLink->GetName().c_str() << std::endl;

	this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint("revolute",this->handModel);

	this->fixedJoint->Load(palm_link, attachLink, math::Pose());
    this->fixedJoint->Init();
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);
	this->jointAttached = true;
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraGameController::DetachJoint()
{
	std::cout << "Detaching Fixed Joint! " <<  std::endl;

	this->fixedJoint->Reset();
	this->fixedJoint->Detach();
	this->fixedJoint->Fini();

	this->jointAttached = false;
}



