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

#ifndef SPAWN_LIQUID_PLUGIN_HH
#define SPAWN_LIQUID_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace sim_games
{

	/// \brief class SpawnLiquid
	class SpawnLiquid : public gazebo::WorldPlugin
	{
		/// \brief Constructor
		public: SpawnLiquid();

		/// \brief Destructor
		public: virtual ~SpawnLiquid();

		/// \brief Load plugin
		protected: virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

		/// \brief Get the SDF plugin parameters
		private: void GetParamAndSpawnLiquid(const sdf::ElementPtr _sdf);

		/// \brief Generate liquid sdf
		private: sdf::SDF GenerateLiquidSDF(
				const gazebo::math::Vector3 _init_pos,
				const unsigned int _nr_spheres,
				const double _sphere_radius,
				const double _spawn_diam,
				const double _inertia,
				const double _mass,
				const double _mu,
				const double _mu2,
				const double _slip1,
				const double _slip2,
				const bool _auto_disable
				);

		/// \brief Arrange spheres in a circular fashion
		private: gazebo::math::Vector3 ArrangeSphere(
				const int _curr_sphere_index,
				const double _radius,
				const double _spawn_diameter,
				int& _spawned,
				int& _level);

		/// \brief World pointer
		private: gazebo::physics::WorldPtr world;

	};
}
#endif
