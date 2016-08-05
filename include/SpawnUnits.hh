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

#ifndef SPAWN_UNITS_PLUGIN_HH
#define SPAWN_UNITS_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace sim_games
{

/// \brief class SpawnUnits
class SpawnUnits : public gazebo::WorldPlugin
{
    /// \brief Constructor
    public: SpawnUnits();

    /// \brief Destructor
    public: virtual ~SpawnUnits();

    /// \brief Load plugin
    protected: virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Generate sdf
    private: sdf::SDF GenerateSDF(
            const std::string _model_name,
            const std::string _spawn_geometry,
            const std::string _vis_geometry,
            const double _unit_nr,
            const double _spawn_diam,
            const gazebo::math::Vector3 _spawn_pos,
            const gazebo::math::Vector3 _inertia_vect,
            const double _unit_mass,
            const double _unit_size,
            const double _unit_length,
            const double _cfm,
            const double _erp,
            const double _kp,
            const double _kd,
            const std::string _collide_bitmask,
            const std::string _scripts_uri,
            const std::string _textures_uri,
            const std::string _mesh_uri,
            const double _mesh_scale,
            const std::string _script_name,
            const bool _static_flag
            );

    /// \brief Arrange spheres in a circular fashion
    private: gazebo::math::Vector3 ArrangeUnits(
            const int _curr_sphere_index,
            const double _radius,
            const double _spawn_diameter,
            int& _spawned,
            int& _level);

};

}

#endif
