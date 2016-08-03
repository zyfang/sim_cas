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

#include "HydraFreezeModels.hh"
using namespace gazebo;
using namespace sim_games;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(HydraFreezeModels)

//////////////////////////////////////////////////
HydraFreezeModels::HydraFreezeModels()
{
    this->toggleBtnPressed = false;
    this->toggleBtnPressed2 = false;
    this->modelsInit = false;
    this->unfreezeIter = 0;
}

//////////////////////////////////////////////////
HydraFreezeModels::~HydraFreezeModels()
{
}

//////////////////////////////////////////////////
void HydraFreezeModels::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world = _parent;

    this->sdf = _sdf;

    // Initialize the transport node
    this->gznode = transport::NodePtr(new transport::Node());
    this->gznode->Init(_parent->GetName());

    // Subscribe to hydra topic
    this->hydraSub = this->gznode->Subscribe("~/hydra",
            &HydraFreezeModels::OnHydra, this);

    std::cout << "******** HYDRA FREEZE MODELS LOADED *********" << std::endl;
}

/////////////////////////////////////////////////
void HydraFreezeModels::OnHydra(ConstHydraPtr &_msg)
{
    // toggle button presssed
    if (_msg->right().button_4())
    {
        // button pressed
        this->toggleBtnPressed = true;
    }

    // if button released toggle pause hand button
    if(this->toggleBtnPressed && !_msg->right().button_4())
    {
        std::cout << "**********************Click" << std::endl;
        // set button to released, and toggle pause hand
        this->toggleBtnPressed = false;

        // init models vector and set them all to static
        if (!this->modelsInit)
        {
            // models init
            this->modelsInit = true;

            // get the models from sdf
            if (this->sdf->HasElement("model"))
            {
                //  get model element
                sdf::ElementPtr elem = this->sdf->GetElement("model");

                while (elem)
                {
                    physics::ModelPtr m = this->world->GetModel(elem->Get<std::string>());                    

                    if (!m->IsStatic())
                    {
                        // set model to static
                        m->SetStatic(true);
                        std::cout << "*INIT* " << m->GetName() << " set to static: " << m->IsStatic() << std::endl;
                    }

                    // add model to list
                    this->models.push_back(m);

                    // get next model element
                    elem = elem->GetNextElement("model");
                }
            }
        }
        // unfreeze one model at a time
        else
        {
            // in case we only have one model
            if(this->models.size() == 1)
            {
                if(this->models.back()->IsStatic())
                {
                    this->models.back()->SetStatic(false);
                    std::cout << "Unfreeze : " << this->models.back()->GetName() << std::endl;
                }
                else
                {
                   this->models.back()->SetStatic(true);
                    std::cout << "Set to static : " << this->models.back()->GetName() << std::endl;
                }
            }
            else
            {                
                if(this->unfreezeIter == 0)
                {
                    this->models[this->unfreezeIter]->SetStatic(false);
                    std::cout << this->models[this->unfreezeIter]->GetName() << " set to movable! "<< std::endl;

                    if(!this->models.back()->IsStatic())
                    {
                        this->models.back()->SetStatic(true);
                        std::cout << this->models.back()->GetName() << " set to static! "<< std::endl;
                    }
                }
                else
                {
                    this->models[this->unfreezeIter]->SetStatic(false);
                    std::cout << this->models[this->unfreezeIter]->GetName() << " set to movable! "<< std::endl;
                    this->models[this->unfreezeIter - 1]->SetStatic(true);
                    std::cout << this->models[this->unfreezeIter - 1]->GetName() << " set to static! "<< std::endl;
                }

                this->unfreezeIter = (this->unfreezeIter + 1) % this->models.size();
            }
        }
    }
}

/////////////////////////////////////////////////
void HydraFreezeModels::OnHydra2(ConstHydraPtr &_msg)
{
    // toggle button presssed
    if (_msg->right().button_4())
    {
        // button pressed
        this->toggleBtnPressed = true;
    }

    // if button released toggle pause hand button
    if(this->toggleBtnPressed && !_msg->right().button_4())
    {
        std::cout << "**********************Click Making Static" << std::endl;
        // set button to released, and toggle pause hand
        this->toggleBtnPressed = false;

        this->world->GetModel("Champignon")->SetStatic(true);
    }


    // toggle button presssed
    if (_msg->right().button_2())
    {
        // button pressed
        this->toggleBtnPressed2 = true;
    }

    // if button released toggle pause hand button
    if(this->toggleBtnPressed2 && !_msg->right().button_2())
    {
        std::cout << "**********************Click2 Making Movable" << std::endl;
        // set button to released, and toggle pause hand
        this->toggleBtnPressed2 = false;

        this->world->GetModel("Champignon")->SetStatic(false);
    }
}
