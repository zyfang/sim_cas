/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Andrei Haidu, Institute for Artificial Intelligence,
 *  Universität Bremen.
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

#include "ArrangeModels.hh"


using namespace gazebo;

// define random gen
#define get_rand(min, max) ((double)rand() / (double)RAND_MAX) * (max - min) + min

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ArrangeModels)

//////////////////////////////////////////////////
ArrangeModels::ArrangeModels()
{
	// initialize random seed
	srand(time(NULL));
}

//////////////////////////////////////////////////
ArrangeModels::~ArrangeModels()
{

}

//////////////////////////////////////////////////
void ArrangeModels::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
	std::cout << "******** ARRANGING MODELS PLUGIN LOADED *********" << std::endl;

	// create the config
	libconfig::Config cfg;

	// models that are already arranged
	std::vector<physics::ModelPtr> arranged_models;

	// read config file
	cfg.readFile("arrange_models.cfg");

	// get the parent model
	physics::ModelPtr parent_model = _world->GetModel(cfg.lookup("onTop.parent").c_str());

	// get the boundingbox of the parent
	math::Box parent_bb = parent_model->GetBoundingBox();

	// get the static models
	libconfig::Setting& static_names = cfg.lookup("onTop.static");

	// add the static models to the arranged models
	for(int i = 0; i < static_names.getLength(); ++i)
	{
		// get the parent model
		arranged_models.push_back(_world->GetModel(static_names[i].c_str()));
	}

	// get the models to be randomized
	libconfig::Setting& random_names = cfg.lookup("onTop.random");

	for(int i = 0; i < random_names.getLength(); ++i)
	{
		// random position
		math::Vector3 rand_pos;

		// get the parent model
		physics::ModelPtr r_model = _world->GetModel(random_names[i].c_str());

		// get the current model bb
		math::Box r_model_bb = r_model->GetBoundingBox();

		// offsets for the given object
		double x_offset = 0.1; //r_model_bb.max.x - r_model_bb.min.x;
		double y_offset = 0.1; //r_model_bb.max.y - r_model_bb.min.y;
		double z_offset = r_model_bb.max.z - r_model_bb.min.z;

		// bool to check if the random position is fine
		bool pos_ok = false;


		std::cout << "RANDOM MODEL: " << r_model->GetName() << std::endl;


		while(!pos_ok)
		{
			// create random position
			// notice that parenthesis are needed for get_rand sums
			rand_pos = math::Vector3(
					get_rand((parent_bb.min.x + x_offset), (parent_bb.max.x - x_offset)),
					get_rand((parent_bb.min.y + y_offset), (parent_bb.max.y - y_offset)),
					parent_bb.max.z + z_offset);

			std::cout << "\t Creating rand pos " << rand_pos << std::endl;

			// flag to check for collision
			bool coll = false;

			// check for collisions with the already arranged models:
			for (int i = 0; i < arranged_models.size(); i++)
			{
				std::cout << "\t\t checking coll with " << arranged_models[i]->GetName() << std::endl;

				// if the two bb are colliding then try another random
				if(ArrangeModels::DoBBIntersect(
						rand_pos.x, rand_pos.y, r_model_bb.GetXLength(), r_model_bb.GetYLength(), arranged_models[i]->GetBoundingBox()))
				{
					std::cout << "\t\t !!!!!!!!! coll detected " << std::endl;
					coll = true;
					continue;
				}
			}

			// if there was no collision the position is fine
			if (!coll)
			{
				std::cout << "\t NO coll detected, position is fine " << std::endl;
				pos_ok = true;
			}
		}



		// set orientation
		const math::Quaternion quat = math::Quaternion(0,0,0);

		// create pose
		const math::Pose rand_pose = math::Pose(rand_pos, quat);

		// set model pose
		r_model->SetWorldPose(rand_pose);

		// add model to the arranged ones
		arranged_models.push_back(r_model);
	}

}

//////////////////////////////////////////////////
bool ArrangeModels::DoBBIntersect(double _pos_x, double _pos_y, double _length_x, double _length_y, math::Box _bb2)
{
	return std::abs(_pos_x - _bb2.GetCenter().x) * 2 < (_length_x + _bb2.GetXLength()) &&
			std::abs(_pos_y - _bb2.GetCenter().y) * 2 < (_length_y + _bb2.GetYLength())  ;
}




















