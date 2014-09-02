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


#include "CreatePancake.hh"

using namespace gazebo;

//////////////////////////////////////////////////
CreatePancake::CreatePancake()
{
	// set up rosnode
	int argc = 0;
	char** argv = NULL;
	ros::init(argc,argv,"Create_pancake_plugin");
	this->rosnode = new ros::NodeHandle();

	// subscribe to hydra ros topic
	this->rosSubscriber = this->rosnode->subscribe(
			"/hydra_raw", 10, &CreatePancake::HydraCallback, this);
}

//////////////////////////////////////////////////
CreatePancake::~CreatePancake()
{
	delete this->rosnode;
}

//////////////////////////////////////////////////
void CreatePancake::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	this->world = _parent;

	// set the parameters from the plugin
	CreatePancake::GetSDFParameters(_sdf);

	// flags for buttons pressed and created pancake
	this->jointButtonPressed = false;
	this->centerAndChildLinksInit = false;
	this->pancakeCreated = false;

	// current counter of spheres
	this->childSphereCounter = 0;

	std::cout << "******** CREATE PANCAKE PLUGIN LOADED *********" << std::endl;
}

//////////////////////////////////////////////////
void CreatePancake::Init()
{
	// no update loop, pancake is not breakable
	if ((this->forceLimit == 0) && (this->distanceLimit) == 0)
	{
//		std::cout << "**Pancake Indestructible" << std::endl;
	}

	// update loop for the case when the pancake is breakable with force limits
	else if ((this->forceLimit > 0) && (this->distanceLimit) == 0)
	{
//		std::cout << "**Pancake breakable with force limit: " << this->forceLimit << std::endl;

		// update and check for distance limit breaches
		this->worldConnection = event::Events::ConnectWorldUpdateBegin(
		  boost::bind(&CreatePancake::CheckJointForceLimits, this));
	}

	// update loop for the case when the pancake is breakable with distance limits
	else if ((this->forceLimit == 0) && (this->distanceLimit) > 0)
	{
//		std::cout << "**Pancake breakable with distance limit: " << this->distanceLimit << std::endl;

		// update and check for distance limit breaches
		this->worldConnection = event::Events::ConnectWorldUpdateBegin(
		  boost::bind(&CreatePancake::CheckJointDistanceLimits, this));
	}

	// update loop for the case when the pancake is breakable with force or distance limits
	else if ((this->forceLimit > 0) && (this->distanceLimit) > 0)
	{
//		std::cout << "**Pancake breakable with distance limit: " << this->distanceLimit
//				<< ", and force limit: " << this->forceLimit << std::endl;

		// update and check for both joint limit breaches
		this->worldConnection = event::Events::ConnectWorldUpdateBegin(
		  boost::bind(&CreatePancake::CheckJointLimits, this));
	}
}

//////////////////////////////////////////////////
void CreatePancake::CheckJointLimits()
{
	// iterate through all the joints and check if limits have been breached
    for (int i=0; i < this->pancakeJoints.size(); ++i)
    {
    	// check force limits
    	if (this->pancakeJoints[i]->GetLinkForce(0).GetLength() > this->forceLimit ||
    			this->pancakeJoints[i]->GetAnchorErrorPose().pos.GetLength() > this->distanceLimit)
    	{
    		std::cout << "Breaking sphere! Force: "	<< this->pancakeJoints[i]->GetLinkForce(0).GetLength()
    				<< ", Distance: " << this->pancakeJoints[i]->GetAnchorErrorPose().pos.GetLength() << std::endl;

    		// break the joint of the pancake
    		CreatePancake::BreakPancakeJoint(i);
    	}
    }
}

//////////////////////////////////////////////////
void CreatePancake::CheckJointForceLimits()
{
	// iterate through all the joints and check if limits have been breached
    for (int i=0; i < this->pancakeJoints.size(); ++i)
    {
    	// check force limits
    	if (this->pancakeJoints[i]->GetLinkForce(0).GetLength() > this->forceLimit)
    	{
    		std::cout << "Breaking sphere! Force: "	<< this->pancakeJoints[i]->GetLinkForce(0).GetLength()
    				<< ", Distance: " << this->pancakeJoints[i]->GetAnchorErrorPose().pos.GetLength() << std::endl;

    		// break the joint of the pancake
    		CreatePancake::BreakPancakeJoint(i);
    	}
    }
}

//////////////////////////////////////////////////
void CreatePancake::CheckJointDistanceLimits()
{
	// iterate through all the joints and check if limits have been breached
    for (int i=0; i < this->pancakeJoints.size(); ++i)
    {
    	// check force limits
    	if (this->pancakeJoints[i]->GetLinkForce(0).GetLength() > this->forceLimit ||
    			this->pancakeJoints[i]->GetAnchorErrorPose().pos.GetLength() > this->distanceLimit)
    	{
    		std::cout << "Breaking sphere! Force: "	<< this->pancakeJoints[i]->GetLinkForce(0).GetLength()
    				<< ", Distance: " << this->pancakeJoints[i]->GetAnchorErrorPose().pos.GetLength() << std::endl;

    		// break the joint of the pancake
    		CreatePancake::BreakPancakeJoint(i);
    	}
    }
}

//////////////////////////////////////////////////
void CreatePancake::HydraCallback(const hand_sim_msgs::HydraRaw& _msg)
{
	// if all the links are connected (the pancake is created) return from callback
	if (this->pancakeCreated)
	{
		// shut down ros subscriber and node
		this->rosSubscriber.shutdown();

		this->rosnode->shutdown();

		return;
	}

	// check if button pressed
	if (_msg.buttons[1] == 1)
	{
		this->jointButtonPressed = true;
	}

	// check if button was released
	if(this->jointButtonPressed && _msg.buttons[1] == 0)
	{
		// set button state to released
		this->jointButtonPressed = false;

		// TODO fix this for loading it in Init() or Load() or smth
		// load the liquid model
		if(this->liquidModel == NULL)
		{
			this->liquidModel = this->world->GetModel("liquid_spheres");
		}

		// check if center and child links are initialised
		if (!this->centerAndChildLinksInit)
		{
			// initalise center and child sphere links
			CreatePancake::InitCenterAndChildLinks();

			// set flag to true
			this->centerAndChildLinksInit = true;

			// create the pancake by adding the dynamic joints between  the links
			CreatePancake::ConnectPancake();
		}
	}
}

//////////////////////////////////////////////////
void CreatePancake::InitCenterAndChildLinks()
{
	// point cloud that will take the positions of the spheres as values
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// create point cloud with the position of the spheres
	CreatePancake::InitSphereCloud(cloud);

	// input cloud, output center pos
	pcl::PointXYZ center_pos = CreatePancake::GetCloudCentroid(cloud);

//	for (size_t i = 0; i < indices.size (); ++i)
//	{
//		centroid[0] += cloud[indices[i]].x;
//		centroid[1] += cloud[indices[i]].y;
//		centroid[2] += cloud[indices[i]].z;
//	}
//	centroid /= nr_spheres;

	// get center link index, by computing closest point from the centroid
	unsigned int center_index = CreatePancake::GetClosestPoint(cloud, center_pos);


	std::stringstream ss;
	// set the center link
	ss << "sphere_link_" << center_index;
	this->centerLink = this->liquidModel->GetLink(ss.str());
	ss.str("");

	// set all the child links
	for (unsigned int i=0; i < cloud.size(); i++)
	{
		if (i != center_index)
		{
			ss << "sphere_link_" << i;
			this->childLinks.push_back(this->liquidModel->GetLink(ss.str()));
			ss.str("");
		}
	}
}

//////////////////////////////////////////////////
void CreatePancake::CreateDynamicJoint(
		physics::LinkPtr _center_link, physics::LinkPtr _ext_link)
{
//	std::cout << "Creating joint between parent link: " << _center_link->GetName() <<
//			" and child link: " << _ext_link->GetName() << std::endl;

	math::Vector3 axis, direction;
	physics::JointPtr joint;

	// direction is the vector between the two spheres
	direction = _ext_link->GetWorldPose().pos - _center_link->GetWorldPose().pos;
	direction.Normalize();

	// the axis is the perpedicular on the direction, so the spheres rotate around the center
	axis = direction.Cross(math::Vector3(0.0, 0.0, 1.0));

	// create joint
	joint = this->world->GetPhysicsEngine()->CreateJoint(
			"revolute", this->liquidModel);

	// Pose is the offset from the child, that is why we leave it to 0
	// Pose containing Joint Anchor offset from child link.
	// setting anchor relative to gazebo child link frame position
	joint->Load(_ext_link, _center_link, math::Pose());


    // child and parent links are switched so the joint ends up on the parent
	joint->Attach(_ext_link, _center_link);

	// set the axis of rotation relative to the local frame
	joint->SetAxis(0, axis);

	// set limits and other parameters
	joint->SetHighStop(0, this->highStop);
	joint->SetLowStop(0, this->lowStop);

//	joint->SetDamping(0, this->damping);
//	joint->SetStiffness(0, this->stiffness);


	// reducing the error reductions parameter allows joint to exceed the limit
	joint->SetParam("cfm", 0, this->cfm);
	joint->SetParam("erp", 0, this->erp);
	joint->SetParam("stop_cfm", 0, this->cfm);
	joint->SetParam("stop_erp", 0, this->erp);

	// fudge factor is used to scale this excess force.
	// It should have a value between zero and one (the default value).
	// If the jumping motion is too visible in a joint, the value can be reduced.
	// Making this value too small can prevent the motor from being able to move the joint away from a stop.
//	joint->SetParam("fudge_factor", 0, this->fudgeFactor);


	joint->SetProvideFeedback(true);

	this->pancakeJoints.push_back(joint);

}

//////////////////////////////////////////////////
void CreatePancake::InitSphereCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	std::stringstream ss;
	unsigned int i = 0;
	ss << "sphere_link_" << i;

	while (this->liquidModel->GetLink(ss.str()) != NULL)
	{
		math::Vector3 pos = this->liquidModel->GetLink(ss.str())->GetWorldPose().pos;
		cloud.push_back(pcl::PointXYZ(pos.x, pos.y, pos.z));
		i++;
		ss.str("");
		ss << "sphere_link_" << i;
	}
}

//////////////////////////////////////////////////
pcl::PointXYZ CreatePancake::GetCloudCentroid(
		pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	Eigen::Vector4f centroid;
	pcl::PointXYZ center_pos;

	pcl::compute3DCentroid(cloud, centroid);

	center_pos.x = centroid[0];
	center_pos.y = centroid[1];
	center_pos.z = centroid[2];

	return center_pos;
}

//////////////////////////////////////////////////
unsigned int CreatePancake::GetClosestPoint(
		pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ pos)
{
	pcl::KdTree<pcl::PointXYZ>::Ptr tree;

	cloud.push_back(pos);

	tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
	tree->setInputCloud(cloud.makeShared());

	int k = 2;
	std::vector<int> k_indices;
	std::vector<float> k_distances;
	k_indices.resize(k);
	k_distances.resize(k);

	//THE CENTER IS THE NEAREST NEIGHBOUR OF THE ADDED POINT, WHICH IS DELETED AFTERWARDS
	//last point of the cloud (the center)
	int index = (int)cloud.points.size()-1;
	tree->nearestKSearch(index, k, k_indices, k_distances);

	cloud.points.pop_back();

	//ROS_INFO("center indice: %u",k_indices[1]);

	return k_indices[1];
}

//////////////////////////////////////////////////
void CreatePancake::ConnectPancake()
{

	// update pancake inertia, the rest are updated in the creation
	this->centerLink->GetInertial()->SetInertiaMatrix(
			this->pancakeInertia, this->pancakeInertia, this->pancakeInertia, 0, 0, 0);
	// to apply the new inertia UpdateMass needs to be called
	this->centerLink->UpdateMass();

	// create pancake, add dynamic joints between links
	for (unsigned int i = 0; i < this->childLinks.size(); i++)
	{
		// create dynamic joint
		CreatePancake::CreateDynamicJoint(
				this->centerLink, this->childLinks[i]);

		// update child link inertia
		this->childLinks[i]->GetInertial()->SetInertiaMatrix(
				this->pancakeInertia, this->pancakeInertia, this->pancakeInertia, 0, 0, 0);
		// to apply the new inertia UpdateMass needs to be called
		this->childLinks[i]->UpdateMass();

	}

	// Set pancake created flag to true
	this->pancakeCreated = true;

	std::cout << "Pancake Created!" << std::endl;

	// Dynamically change the surface parameters in order to harden it
	// todo
//	const std::vector<physics::LinkPtr> links = this->liquidModel->GetLinks();
//	for (unsigned int i = 0; i < links.size(); ++i)
//	{
//		std::cout << "ORIGINAL link " << i << " kp = " << links[i]->GetCollision(0)->GetSurface()->kp <<
//				" kd = " << links[i]->GetCollision(0)->GetSurface()->kd << std::endl;
//		links[i]->GetCollision(0)->GetSurface()->kp = 10000000;
//		links[i]->GetCollision(0)->GetSurface()->kd = 100;
//		links[i]->UpdateSurface();
//	}
//	for (unsigned int i = 0; i < links.size(); ++i)
//	{
//		std::cout << "CHANGED link " << i << " kp = " << links[i]->GetCollision(0)->GetSurface()->kp <<
//				" kd = " << links[i]->GetCollision(0)->GetSurface()->kd << std::endl;
//	}
//	std::cout << " Pancake surface hardened " << std::endl;


	// Changing the pancakes spheres inertial values
	// these were set to values that made them difficult to roll
	// now it is set back to the given parameters from the sdf file
	// to allow flipping of the pancake

}

//////////////////////////////////////////////////
void CreatePancake::BreakPancakeJoint(unsigned int i)
{
	// joint link 1 is the ext sphere
	physics::LinkPtr ext_sphere = this->pancakeJoints[i]->GetJointLink(1);

	// stop the sphere
	ext_sphere->SetAngularVel(math::Vector3(0,0,0));
	ext_sphere->SetLinearVel(math::Vector3(0,0,0));

	// apply new inertia so the sphere doesn't roll
	ext_sphere->GetInertial()->SetInertiaMatrix(
			1.0, 1.0, 1.0, 0, 0, 0);
	// to apply the new inertia UpdateMass needs to be called
	ext_sphere->UpdateMass();

	// detach the joint
	this->pancakeJoints[i]->Detach();

	// stop providing feedback from the joint
	this->pancakeJoints[i]->SetProvideFeedback(false);

	// remove traces of the joint
	this->pancakeJoints[i]->Fini();

	// remove joint from the vector
	this->pancakeJoints.erase(this->pancakeJoints.begin() + i);
}

//////////////////////////////////////////////////
void CreatePancake::GetSDFParameters(const sdf::ElementPtr _sdf)
{
	////////////// stop_cfm
	if (!_sdf->HasElement("stop_cfm"))
	{
		std::cout << "Missing parameter <stop_cfm> in DynamicJoint, default to 0" << std::endl;
		this->stopCfm = 0;
	}
	else this->stopCfm = _sdf->Get<double>("stop_cfm");

	////////////// stop_erp
	if (!_sdf->HasElement("stop_erp"))
	{
		std::cout << "Missing parameter <stop_erp> in DynamicJoint, default to 0" << std::endl;
		this->stopErp = 0;
	}
	else this->stopErp = _sdf->Get<double>("stop_erp");

	////////////// fudge_factor
	if (!_sdf->HasElement("fudge_factor"))
	{
		std::cout << "Missing parameter <fudge_factor> in DynamicJoint, default to 0" << std::endl;
		this->fudgeFactor = 0;
	}
	else this->fudgeFactor = _sdf->Get<double>("fudge_factor");

	////////////// cfm
	if (!_sdf->HasElement("cfm"))
	{
		std::cout << "Missing parameter <cfm> in DynamicJoint, default to 0" << std::endl;
		this->cfm = 0;
	}
	else this->cfm = _sdf->Get<double>("cfm");

	////////////// erp
	if (!_sdf->HasElement("erp"))
	{
		std::cout << "Missing parameter <erp> in DynamicJoint, default to 0" << std::endl;
		this->erp = 0;
	}
	else this->erp = _sdf->Get<double>("erp");

	////////////// hi stop
	if (!_sdf->HasElement("highStop"))
	{
		std::cout << "Missing parameter <highStop> in DynamicJoint, default to 0" << std::endl;
		this->highStop = 0;
	}
	else this->highStop = _sdf->Get<double>("highStop");

	////////////// low stop
	if (!_sdf->HasElement("lowStop"))
	{
		std::cout << "Missing parameter <lowStop> in DynamicJoint, default to 0" << std::endl;
		this->lowStop = 0;
	}
	else this->lowStop = _sdf->Get<double>("lowStop");

	////////////// pancake_inertia
	if (!_sdf->HasElement("pancake_inertia"))
	{
		std::cout << "Missing parameter <pancake_inertia> in DynamicJoint, default to 0.00002" << std::endl;
		this->pancakeInertia = 0.00002;
	}
	else this->pancakeInertia = _sdf->Get<double>("pancake_inertia");

	////////////// damping
	if (!_sdf->HasElement("damping"))
	{
		std::cout << "Missing parameter <damping> in DynamicJoint, default to 0" << std::endl;
		this->damping = 0;
	}
	else this->damping = _sdf->Get<double>("damping");

	////////////// stiffness
	if (!_sdf->HasElement("stiffness"))
	{
		std::cout << "Missing parameter <stiffness> in DynamicJoint, default to 0.00002" << std::endl;
		this->stiffness = 0.00002;
	}
	else this->stiffness = _sdf->Get<double>("stiffness");

	////////////// force_limit
	if (!_sdf->HasElement("force_limit"))
	{
		std::cout << "Missing parameter <force_limit> in DynamicJoint, default to 0 (disabled)" << std::endl;
		this->forceLimit = 0.0;
	}
	else this->forceLimit = _sdf->Get<double>("force_limit");

	////////////// distance_limit
	if (!_sdf->HasElement("distance_limit"))
	{
		std::cout << "Missing parameter <distance_limit> in DynamicJoint, default to 0 (disabled)" << std::endl;
		this->distanceLimit = 0.0;
	}
	else this->distanceLimit = _sdf->Get<double>("distance_limit");

}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CreatePancake)


