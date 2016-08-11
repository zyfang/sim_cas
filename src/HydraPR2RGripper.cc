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

#include "HydraPR2RGripper.hh"
#include <boost/tokenizer.hpp>
#include <gazebo/transport/transport.hh>
#include "GzUtils.hh"

#define PI 3.14159265359

using namespace sim_games;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HydraPR2RGripper)


//////////////////////////////////////////////////
HydraPR2RGripper::HydraPR2RGripper() : offsetPos(0.8, 0, 0.8),
    offsetQuat(PI, 0, 0)
{
    // init hand movement state flag
    this->disableHydra = true;

    // init pause button state flag
    this->disableBtnPressed = false;

    // set flag to false
    this->loggingOn = false;

    // initialize gripper state flags
    this->idleGripper = true;
    this->closingGripper = false;
    this->jointAttached = false;
    this->rTipInContact = false;
    this->lTipInContact = false;

    // set different initial values to the model in contact with grippers
    this->rContactModelName = " 1";
    this->lContactModelName = " 2";

    // Set up control and finger joint PIDs
    double const _control_P = 100;
    double const _control_I = 0;
    double const _control_D = 50;

    // add pid controllers for pos
    this->posPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
    this->posPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
    this->posPIDs.push_back(common::PID(_control_P, _control_I, _control_D, 1000, -1000));
}

//////////////////////////////////////////////////
HydraPR2RGripper::~HydraPR2RGripper()
{
}

//////////////////////////////////////////////////
void HydraPR2RGripper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // create the config
    libconfig::Config cfg;  
    // get the config file path, default log_location.cfg
    const std::string config_file =
            GetSDFValue(std::string("config_file"), _sdf, std::string("config/log_location.cfg"));

    // read config file
    try
    {
        cfg.readFile(config_file.c_str());
    }
    catch(const libconfig::FileIOException &fioex)
    {
        std::cerr << "I/O error while reading file. " << config_file.c_str() << std::endl;
    }
    catch(const libconfig::ParseException &pex)
    {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                                << " - " << pex.getError() << std::endl;
    }
    // get top folder name
    std::string topfolder = cfg.lookup("logfolder.log");
    // get the exp ID
    std::string expID = cfg.lookup("logfolder.exp");
    // get the subject ID
    std::string subjID = cfg.lookup("logfolder.subject");
    // get the demonstration ID
    std::string demoID = cfg.lookup("logfolder.isim");
    // put logpath together
    this->logpath = topfolder + "/" + expID + "/" + subjID + "/" + demoID;
    
    ///////////////////////////////////////////////////////////
    // Store the pointer to the model
    this->gripperModel = _parent;

    // disable gravity on the hand model
    this->gripperModel->SetGravityMode(false);

    // initialize world
    this->world = this->gripperModel->GetWorld();

    // get hand joint controller (in case of position control)
    this->jointController = this->gripperModel->GetJointController();

    // set joint init position (in case of position control)
    this->jointController->SetPositionTarget(
                "PR2RGripper::r_gripper_r_screw_screw_joint", 0);

    // initialize the gripper screw joints (in case of force control)
//    this->rScrewJoint = this->gripperModel->GetJoint("r_gripper_r_screw_screw_joint");
//    this->lScrewJoint = this->gripperModel->GetJoint("r_gripper_l_screw_screw_joint");

    // Initialize the transport node
    this->gznode = transport::NodePtr(new transport::Node());

    this->gznode->Init(this->world->GetName());

    // Subscribe to hydra topic
    this->hydraSub = this->gznode->Subscribe("~/hydra",
            &HydraPR2RGripper::OnHydra, this);

    // "~/PR2Gripper/r_gripper_l_finger_tip_link/r_gripper_l_finger_tip_contact_sensor/contacts"
    // Subscribe to the right finger tip contact sensor
    this->rGripperSub = this->gznode->Subscribe("~/r_gripper_r_finger_tip_contact",
            &HydraPR2RGripper::OnRGripperContact, this);

    // Subscribe to the left finger tip contact sensor
    this->lGripperSub = this->gznode->Subscribe("~/r_gripper_l_finger_tip_contact",
            &HydraPR2RGripper::OnLGripperContact, this);
    
    // Publish custom event msgs
    this->customEventPub = this->gznode->Advertise<msgs::GzString>("~/custom_events");

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&HydraPR2RGripper::OnUpdate, this));

    // set desired position the current hand position (the spawned position)
    this->desiredPosition = this->gripperModel->GetWorldPose().pos;

    // set the initial value of the rotation
    this->desiredQuat = this->gripperModel->GetWorldPose().rot;

    std::cout << "******** HYDRA PR2 GRIPPER PLUGIN LOADED *********" << std::endl;
}

//////////////////////////////////////////////////
void HydraPR2RGripper::OnUpdate()
{
    // compute step time
    common::Time step_time = this->world->GetSimTime() - this->prevSimTime;
    this->prevSimTime = this->world->GetSimTime();

    // brief apply forces in order to control the hand Pose
    HydraPR2RGripper::GripperPoseControl(step_time);
}

/////////////////////////////////////////////////
void HydraPR2RGripper::OnHydra(ConstHydraPtr &_msg)
{
    // check disable button status
    if (_msg->right().button_center())
    {
        // button pressed
        this->disableBtnPressed = true;
    }

    // if button released toggle pause hand button
    if(this->disableBtnPressed && !_msg->right().button_center())
    {
        // set button to released, and toggle pause hand
        this->disableBtnPressed = false;
        this->disableHydra = !this->disableHydra;
    }

    // if hand is paused return from the callback
    if (this->disableHydra)
        return;

    // update the desired pose of the hand
    HydraPR2RGripper::UpdateGripperPose(_msg);

    // brief control fingers
    HydraPR2RGripper::GripperControl(_msg->right().joy_x());

    // toggle between logging the world
    HydraPR2RGripper::ToggleLogging(_msg->right().button_3());
}

////////////////////////////////////////////////////
void HydraPR2RGripper::GripperPoseControl(const common::Time _step_time)
{
    // get current pose of the hand
    const math::Pose curr_pose = this->gripperModel->GetWorldPose();

    // the translation and rotation values to be applied on the hand
    math::Vector3 pos_effort, rot_effort;

    // PID values for hand control
    // computing PID values from the current position, for the x, y, z axis
    pos_effort.x = this->posPIDs[0].Update(curr_pose.pos.x - this->desiredPosition.x, _step_time);
    pos_effort.y = this->posPIDs[1].Update(curr_pose.pos.y - this->desiredPosition.y, _step_time);
    pos_effort.z = this->posPIDs[2].Update(curr_pose.pos.z - this->desiredPosition.z, _step_time);

    // compute rotation velocity values from the current quaternion
    rot_effort = HydraPR2RGripper::ReturnRotVelocity(curr_pose.rot);

    // apply the computed lin/rot forces/velocities to the model
    this->gripperModel->GetLink("r_wrist_roll_link")->SetForce(pos_effort);
    this->gripperModel->SetAngularVel(rot_effort);
}


//////////////////////////////////////////////////
void HydraPR2RGripper::GripperControl(const double _joy_x)
{
    // close gripper, _joy_x = [-1,1]
    if (_joy_x > 0 && !this->jointAttached)
    {
        // set the state flags
        this->idleGripper = false;

        // gripper closing
        this->closingGripper = true;

        // pos control
        const double gripper_pos = this->jointController->GetPositions()[
                "PR2RGripper::r_gripper_r_screw_screw_joint"] + ( - _joy_x / 15);

        // move gripper if only between the joint limits
        if (gripper_pos < 150 && gripper_pos > 0.0)
        {
            this->jointController->SetPositionTarget(
                        "PR2RGripper::r_gripper_r_screw_screw_joint", gripper_pos);
        }

        // in case of force control
        /*this->rScrewJoint->SetForce(0, - _joy_x * 2);
        this->lScrewJoint->SetForce(0, - _joy_x * 2);*/
    }
    // open gripper
    else if (_joy_x < 0)
    {
        // set the state flags
        this->idleGripper = false;

        // gripper closing
        this->closingGripper = false;

        // detach joint if attached
        if(this->jointAttached)
        {
            HydraPR2RGripper::DetachJoint();
        }

        // pos control
        const double gripper_pos = this->jointController->GetPositions()[
                "PR2RGripper::r_gripper_r_screw_screw_joint"] + ( - _joy_x / 10);

        // move gripper if only between the joint limits
        if (gripper_pos < 150 && gripper_pos > 0.0)
        {
            this->jointController->SetPositionTarget(
                        "PR2RGripper::r_gripper_r_screw_screw_joint", gripper_pos);
        }

        // in case of force control
        /*this->rScrewJoint->SetForce(0, - _joy_x * 2);
        this->lScrewJoint->SetForce(0, - _joy_x * 2);*/
    }
    // no movement
    else
    {
        // gripper is idle
        this->idleGripper = true;

        // gripper closing
        this->closingGripper = false;
    }
}

//////////////////////////////////////////////////
math::Vector3 HydraPR2RGripper::ReturnRotVelocity(math::Quaternion _curr_quat)
{
    math::Vector3 rot_velocity;
    math::Quaternion quatern_diff, vel_q;

    quatern_diff = (this->desiredQuat - _curr_quat) * 2.0;
    vel_q = quatern_diff * _curr_quat.GetInverse();

    rot_velocity.x = vel_q.x * 8;
    rot_velocity.y = vel_q.y * 8;
    rot_velocity.z = vel_q.z * 8;

    return rot_velocity;
}

//////////////////////////////////////////////////
void HydraPR2RGripper::UpdateGripperPose(ConstHydraPtr &_msg)
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

//////////////////////////////////////////////////
void HydraPR2RGripper::OnRGripperContact(ConstContactsPtr &_msg)
{
    // if gripper is not closing/opening
    if (this->idleGripper)
    {
        return;
    }
    // check for contact msgs
    else if(_msg->contact_size() > 0 && this->closingGripper
            && !this->jointAttached)
    {
        // we only take the first contact into consideration
        //for (unsigned int i = 0; i < _msg->contact_size(); ++i)

        // get the first collision and its corresponding model name
        const std::string c1 = _msg->contact(0).collision1();
        const std::string m1 = c1.substr(0,c1.find("::"));

        // check if first collision is itself
        if (m1 == this->gripperModel->GetName())
        {
            // set the model name of the second collision
            const std::string c2 = _msg->contact(0).collision2();
            this->rContactModelName = c2.substr(0,c2.find("::"));
        }
        else
        {
            // set the model name from the first collision
            this->rContactModelName = m1;
        }

        // attach joint if the two finger tips are in contact with the same model
        if (this->rContactModelName == this->lContactModelName)
        {
            HydraPR2RGripper::AttachJoint();
        }
    }
}

//////////////////////////////////////////////////
void HydraPR2RGripper::OnLGripperContact(ConstContactsPtr &_msg)
{
    // if gripper is not closing/opening
    if (this->idleGripper)
    {
        return;
    }
    // check for contact msgs
    else if(_msg->contact_size() > 0 && this->closingGripper)
    {
        // we only take the first contact into consideration
        //for (unsigned int i = 0; i < _msg->contact_size(); ++i)

        // get the first collision and its corresponding model name
        const std::string c1 = _msg->contact(0).collision1();
        const std::string m1 = c1.substr(0,c1.find("::"));

        // check if first collision is itself
        if (m1 == this->gripperModel->GetName())
        {
            // set the model name of the second collision
            const std::string c2 = _msg->contact(0).collision2();
            this->lContactModelName = c2.substr(0,c2.find("::"));
        }
        else
        {
            // set the model name from the first collision
            this->lContactModelName = m1;
        }

        // attach joint if the two finger tips are in contact with the same model
        if (this->rContactModelName == this->lContactModelName)
        {
            HydraPR2RGripper::AttachJoint();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraPR2RGripper::AttachJoint()
{
    // get first link of the grasped model
    const physics::LinkPtr attach_link =
            this->world->GetModel(this->rContactModelName)->GetLinks().at(0);

    // set gravity off for the grasped model
    this->attachedModel = attach_link->GetModel();
    this->attachedModel->SetGravityMode(false);
    
    const physics::LinkPtr wrist_link =
            this->gripperModel->GetLink("r_wrist_roll_link");

    std::cout << "Creating joint between " << wrist_link->GetName().c_str()
            << " and " << attach_link->GetName().c_str() << std::endl;

    // creating joint
    this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint(
                "revolute", this->gripperModel);

    // attaching and setting the joint to fixed
    this->fixedJoint->Load(wrist_link, attach_link, math::Pose());
    this->fixedJoint->Init();
    this->fixedJoint->SetHighStop(0, 0);
    this->fixedJoint->SetLowStop(0, 0);

    // set attached flag to true
    this->jointAttached = true;

    // set different initial values to the model in contact with grippers
    this->rContactModelName = " 1";
    this->lContactModelName = " 2";
}

//////////////////////////////////////////////////////////////////////////////////////
void HydraPR2RGripper::DetachJoint()
{
    std::cout << "Detaching Fixed Joint! " <<  std::endl;

    // removing and detaching joint
    this->fixedJoint->Reset(); //TODO check if needed
    this->fixedJoint->Detach();
    this->fixedJoint->Fini();
    // this->fixedJoint = nullptr;

    // set gravity on for the detached model
    this->attachedModel->SetGravityMode(true);
    
    // set attached flag to false
    this->jointAttached = false;

    // set different initial values to the model in contact with grippers
    this->rContactModelName = " 1";
    this->lContactModelName = " 2";
}

//////////////////////////////////////////////////
void HydraPR2RGripper::ToggleLogging(const bool _btn)
{
    // toggle between logging the world
    if (_btn)
    {
        // button pressed
        this->logBtnPressed = true;
    }

    if(this->logBtnPressed && !_btn)
    {
        if (!this->loggingOn)
        {
            std::cout << "Starting logging.." << std::endl;

            // set the folder where to save the logs
            util::LogRecord::Instance()->SetBasePath(this->logpath);

            // start recording with given compression type
            util::LogRecord::Instance()->Start("txt"); // txt, bz2, zlib

            // set flag to true
            this->loggingOn = true;
            
            // create event msg
            msgs::GzString gzstr_msg;                
            gzstr_msg.set_data(std::string("start_logging"));
            
            // publish the custom event msg
            this->customEventPub->Publish(gzstr_msg);                    
        }
        else
        {
            std::cout << "Stop logging.." << std::endl;

            // stop recording
            util::LogRecord::Instance()->Stop();

            // set flag to false
            this->loggingOn = false;
            
            // create event msg
            msgs::GzString gzstr_msg;                
            gzstr_msg.set_data(std::string("stop_logging"));
            
            // publish the custom event msg
            this->customEventPub->Publish(gzstr_msg);                       
        }

        // button released
        this->logBtnPressed = false;
    }
}
