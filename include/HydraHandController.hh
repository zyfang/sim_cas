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

#ifndef HYDRA_HAND_CONTROLLER_HH
#define HYDRA_HAND_CONTROLLER_HH

#include <boost/thread/mutex.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/util/LogRecord.hh"

namespace gazebo
{   
	/// \brief HydraHandController class, in controls the robot hand using Hydra as input
	class HydraHandController : public WorldPlugin
	{
		/// \brief Constructor
		public: HydraHandController();

		/// \brief Destructor
		public: virtual ~HydraHandController();

		/// \brief Load plugin
		protected: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

	    /// \brief Update the controller
	    /// \param[in] _info Update information provided by the server.
	    private: void OnUpdate();

	    /// \brief Callback executed every time a new hydra message is received.
	    /// \param[in] _msg The hydra message.
	    private: void OnHydra(ConstHydraPtr &_msg);

		/// \brief Callback thumb contact sensor
		private: void OnThumbContact(ConstContactsPtr &_msg);

		/// \brief Callback fore finger contact sensor
		private: void OnForeFingerContact(ConstContactsPtr &_msg);

		/// \brief apply forces in order to control the hand Pose
		private: void HandPoseControl(const common::Time _step_time);

		/// \brief Update the desired pose of the robot hand
		private: void UpdateDesiredPose(ConstHydraPtr& _msg);

		/// \brief Return the required rotational velocity
		private: math::Vector3 ReturnRotVelocity(const math::Quaternion _curr_quat);

	    /// \brief Pointer to the update event connection
	    private: event::ConnectionPtr updateConnection;

	    /// \brief Node used for using Gazebo communications.
	    private: transport::NodePtr gznode;

	    /// \brief Subscribe to hydra topic.
	    private: transport::SubscriberPtr hydraSub;

		/// \brief thumb contact sensor subscribers
		private: transport::SubscriberPtr thumbContactSub;

		/// \brief fore finger contact sensor subscribers
		private: transport::SubscriberPtr foreContactSub;

	    /// \brief Store the last message from hydra.
	    private: boost::shared_ptr<const gazebo::msgs::Hydra> hydraMsgPtr;

		/// \brief World pointer
	    private: physics::WorldPtr world;

	    /// \brief Model pointer
	    private: physics::ModelPtr handModel;

		/// \brief Positional offset for the hand
		private: const math::Vector3 offsetPos;

		/// \brief Orientation offset for the hand
		private: const math::Quaternion offsetQuat;

		/// \brief Hand joint rotation PID controllers
		private: std::vector<common::PID> rotPIDs;

		/// \brief Hand joint position PID controllers
		private: std::vector<common::PID> controlPIDs;

		/// \brief Hand movement state flag
		private: bool pauseHand;

		/// \brief Hand pause button state
		private: bool pauseButtonPressed;


		/// \brief Desired position of the hand
		private: math::Vector3 desiredPosition;

		/// \brief Desired orientation of the hand
		private: math::Quaternion desiredQuat;

		/// \brief Timestamp of the last cycle for the PID
		private: common::Time prevSimTime;


		/// \brief TODO joint controller
		private: physics::JointControllerPtr jointController;
	};
}
#endif
