/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Andrei Haidu,
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

#ifndef HYDRA_FREEZE_MODELS_HH
#define HYDRA_FREEZE_MODELS_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace sim_games
{

/// \brief class HydraFreezeModels
class HydraFreezeModels : public gazebo::WorldPlugin
{
    /// \brief Constructor
    public: HydraFreezeModels();

    /// \brief Destructor
    public: virtual ~HydraFreezeModels();

    /// \brief Load plugin
    protected: virtual void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra(ConstHydraPtr &_msg);

    /// \brief Callback executed every time a new hydra message is received.
    /// \param[in] _msg The hydra message.
    private: void OnHydra2(ConstHydraPtr &_msg);

    /// \brief Node used for using Gazebo communications.
    private: gazebo::transport::NodePtr gznode;

    /// \brief Subscribe to hydra topic.
    private: gazebo::transport::SubscriberPtr hydraSub;

    /// \brief Pointer to the world
    private: gazebo::physics::WorldPtr world;

    /// \brief Pointer to the sdf
    private: sdf::ElementPtr sdf;

    /// \brief Models to toggle with static flag
    private: std::vector<gazebo::physics::ModelPtr> models;

    /// \brief Models init flag (needed because not all models are directly loaded)
    private: bool modelsInit;

    /// \brief Current model to unfreeze
    private: unsigned int unfreezeIter;

    /// \brief
    private: bool toggleBtnPressed;

    /// \brief
    private: bool toggleBtnPressed2;

};

}

#endif
