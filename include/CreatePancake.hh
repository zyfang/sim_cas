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

#ifndef CREATE_PANCAKE_HH
#define CREATE_PANCAKE_HH

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sstream>


namespace gazebo
{
	// TODO change to a class instead of plugin, to be called from hydra game controller
	/// \brief plugin creating dynamic joints between the liquid spheres
	/// in order to create a pancake
	class CreatePancake : public WorldPlugin
	{
		/// \brief constructor
		public: CreatePancake();

		/// \brief destructor
		public: virtual ~CreatePancake();

		/// \brief load of the function
		protected: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

		/// \brief Init plugin (Load called first, then Init)
		protected: virtual void Init();

	    /// \brief WorldUpdate callback
	    protected: void CheckJointLimits();

	    /// \brief WorldUpdate callback
	    protected: void CheckJointForceLimits();

	    /// \brief WorldUpdate callback
	    protected: void CheckJointDistanceLimits();

		/// \brief callback for every hydra message
		private: void OnHydra(ConstHydraPtr &_msg);

		/// \brief get the SDF plugin parameters
		private: void GetSDFParameters(const sdf::ElementPtr _sdf);

		/// \brief initialize center and child links of the spheres
		private: void InitCenterAndChildLinks();

		/// \brief start creating the joints between the liquid spheres
		private: void ConnectPancake();

		/// \brief create a joint between the anchor link and the exterior link
		private: void CreateDynamicJoint(
				physics::LinkPtr _parent_link, physics::LinkPtr _child_link);

		/// \brief detach joint when force or distance limit is breached
		private: void BreakPancakeJoint(unsigned int i);

		/// \brief pointer to the world
		private: physics::WorldPtr world;

		/// \brief the model of the liquid (the spheres)
		private: physics::ModelPtr liquidModel;

		/// \brief pointer to the center link
		private: physics::LinkPtr centerLink;

		/// \brief vector with the child links
		private: std::vector<physics::LinkPtr> childLinks;

		/// \brief vector of all the joints between the spheres
		private: std::vector<physics::JointPtr> pancakeJoints;

		/// \brief flags used for creating the joints
		private: bool centerAndChildLinksInit, pancakeCreated, jointButtonPressed;

		/// \brief joint parameters
		private: double stopCfm, stopErp, limitStop, pancakeInertia, fudgeFactor, cfm, erp,
		highStop, lowStop, stiffness, damping, forceLimit, distanceLimit;

		/// \brief sphere counter
		private: unsigned int childSphereCounter;

	    /// \brief Pointer to the world update event connection
	    private: event::ConnectionPtr worldConnection;

	    /// \brief Node used for using Gazebo communications.
	    private: transport::NodePtr gznode;

	    /// \brief Subscribe to hydra topic.
	    private: transport::SubscriberPtr hydraSub;

	};
}
#endif
