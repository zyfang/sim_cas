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
HydraHandController::HydraHandController()
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

	// TODO
	this->jointController = this->handModel->GetJointController();



	std::cout << this->jointController->GetJoints().size() << std::endl;


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


	std::cout << "******** HYDRA GAME CONTROLLER LOADED *********" << std::endl;
}

/////////////////////////////////////////////////
void HydraHandController::OnUpdate()
{

}

/////////////////////////////////////////////////
void HydraHandController::OnHydra(ConstHydraPtr &_msg)
{

}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::OnThumbContact(ConstContactsPtr &_msg)
{

}

//////////////////////////////////////////////////////////////////////////////////////
void HydraHandController::OnForeFingerContact(ConstContactsPtr &_msg)
{

}

