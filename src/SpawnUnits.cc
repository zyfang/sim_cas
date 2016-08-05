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

#include "SpawnUnits.hh"
#include "GzUtils.hh"

#define PI 3.14159265

using namespace gazebo;
using namespace sim_games;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SpawnUnits)

//////////////////////////////////////////////////
SpawnUnits::SpawnUnits()
{
}

//////////////////////////////////////////////////
SpawnUnits::~SpawnUnits()
{
}

//////////////////////////////////////////////////
void SpawnUnits::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // model name
    const std::string model_name =
            GetSDFValue(std::string("modelName"), _sdf, std::string("default_name"));

    // spawn type
    const std::string spawn_geometry =
            GetSDFValue(std::string("geometry"), _sdf, std::string("sphere"));

    std::string vis_geometry = spawn_geometry;

    // vis type
    if(_sdf->HasElement("visGeom"))
    {
        vis_geometry = GetSDFValue(std::string("visGeom"), _sdf, spawn_geometry);
    }

    // mesh_uri
    std::string mesh_uri = "";

    // mesh scale
    double mesh_scale = 1.0;

    if (spawn_geometry == "mesh" || vis_geometry == "mesh")
    {
          mesh_uri = GetSDFValue(std::string("meshUri"), _sdf,
                        std::string("model://meshes/meshes/bowl.dae"));

          mesh_scale = GetSDFValue(std::string("meshScale"), _sdf, 1.0);

    }

    // nr of units
    const unsigned int unit_nr =
             GetSDFValue(std::string("unitNr"), _sdf, 1);

    // spawning diam size
    const double spawn_diam =
            GetSDFValue(std::string("spawnDiam"), _sdf, 0.1);

    // spawn pos
    math::Vector3 spawn_pos;

    // if spawn position added
    if(_sdf->HasElement("spawnPos"))
    {
        spawn_pos = GetSDFValue(std::string("spawnPos"), _sdf, math::Vector3(0,0,0));
    }
    // if spawning on top of a model with an offset
    else
    {
        spawn_pos = _parent->GetModel(
                    GetSDFValue(std::string("onTopOfModel"), _sdf, std::string("")))->GetWorldPose().pos
                + GetSDFValue(std::string("onTopOffset"), _sdf, math::Vector3(0,0,0));
    }

    // unit mass
    const double unit_mass =
            GetSDFValue(std::string("unitMass"), _sdf, 0.1);

    // unit size
    const double unit_size =
            GetSDFValue(std::string("unitSize"), _sdf, 0.1);
    
    // unit length
    const double unit_length =
            GetSDFValue(std::string("unitLength"), _sdf, 0.04);

    // inertia vect
    const math::Vector3 inertia_vect =
            GetSDFValue(std::string("inertiaVect"), _sdf, math::Vector3(0.1, 0.1, 0.1));

    // cfm
    const double cfm =
            GetSDFValue(std::string("cfm"), _sdf, 0.0);

    // erp
    const double erp =
            GetSDFValue(std::string("erp"), _sdf, 0.0);

    // kp
    const double kp =
            GetSDFValue(std::string("kp"), _sdf, 0.0);

    // kd
    const double kd =
            GetSDFValue(std::string("kd"), _sdf, 0.0);

    // collide bitmask
    const std::string collide_bitmask =
            GetSDFValue(std::string("collideBitmask"), _sdf, std::string("0x00"));


    // script_uri
    std::string scripts_uri = "";
    if(_sdf->HasElement("scriptsUri"))
    {
           scripts_uri = GetSDFValue(std::string("scriptsUri"), _sdf,
                std::string("file://media/materials/scripts/gazebo.material"));
    }

    // texture_uri
    std::string textures_uri = "";
    if(_sdf->HasElement("texturesUri"))
    {
            textures_uri =GetSDFValue(std::string("texturesUri"), _sdf, std::string(""));
    }

    // script name
    std::string scripts_name = "";
    if(_sdf->HasElement("scriptName"))
    {
            scripts_name = GetSDFValue(std::string("scriptName"), _sdf, std::string("Gazebo/Red"));
    }



    // static flag
    const bool static_flag =
            GetSDFValue(std::string("static"), _sdf, false);


//    std::cout << model_name << std::endl;
//    std::cout << spawn_geometry << std::endl;
//    std::cout << unit_nr << std::endl;
//    std::cout << spawn_diam << std::endl;
//    std::cout << spawn_pos << std::endl;
//    std::cout << inertia_vect << std::endl;
//    std::cout << unit_mass << std::endl;
//    std::cout << unit_size << std::endl;
//    std::cout << cfm << std::endl;
//    std::cout << erp << std::endl;
//    std::cout << kp << std::endl;
//    std::cout << kd << std::endl;
//    std::cout << collide_bitmask << std::endl;
//    std::cout << scripts_uri << std::endl;
//    std::cout << textures_uri << std::endl;
//    std::cout << scripts_name << std::endl;
//    std::cout << static_flag << std::endl;


    // Generate sdf
    const sdf::SDF generatedSDF = SpawnUnits::GenerateSDF(
                model_name, spawn_geometry, vis_geometry, unit_nr, spawn_diam, spawn_pos,
                inertia_vect, unit_mass, unit_size, unit_length, cfm, erp, kp, kd, collide_bitmask,
                scripts_uri, textures_uri, mesh_uri, mesh_scale, scripts_name, static_flag);

    // Insert generated sdf into the world
    _parent->InsertModelSDF(generatedSDF);

    std::cout << "******** " << model_name << " UNITS SPAWNED *********" << std::endl;

}


//////////////////////////////////////////////////
sdf::SDF SpawnUnits::GenerateSDF(
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
        )
{
    std::stringstream sdf_ss;

    // positioning of the current sphere
    math::Vector3 p3;

    // currently spawned spheres
    int spawned = 0;

    // current height level
    int level = 0;


    sdf_ss << "<?xml version='1.0'?>\n";
    sdf_ss << "<sdf version='1.5'>\n";
    sdf_ss << "<model name="<< _model_name <<">\n";
    sdf_ss << "\t<pose>" << _spawn_pos.x << " " << _spawn_pos.y << " " << _spawn_pos.z << " 0 0 0 </pose>\n";

    // Loop for every sphere
    for (unsigned int i = 0; i < _unit_nr; i++)
    {
        // returns the position of the current sphere, spawned and level are returned by reference
        p3 = SpawnUnits::ArrangeUnits(i, _unit_size, _spawn_diam, spawned, level);

        sdf_ss << "\t\t<link name='" << _model_name << "_link_" << i << "'>\n";
        sdf_ss << "\t\t\t<self_collide>true</self_collide>\n";
        sdf_ss << "\t\t\t<pose>" << p3.x << " " << p3.y << " " << p3.z << " 0 0 0</pose>\n";

        sdf_ss << "\t\t\t<inertial>\n";
        sdf_ss << "\t\t\t\t<pose> 0 0 0 0 0 0 </pose>\n";
        sdf_ss << "\t\t\t\t<inertia>\n";
        sdf_ss << "\t\t\t\t\t<ixx>" << _inertia_vect.x << "</ixx>\n";
        sdf_ss << "\t\t\t\t\t<ixy>0</ixy>\n";
        sdf_ss << "\t\t\t\t\t<ixz>0</ixz>\n";
        sdf_ss << "\t\t\t\t\t<iyy>" << _inertia_vect.y << "</iyy>\n";
        sdf_ss << "\t\t\t\t\t<iyz>0</iyz>\n";
        sdf_ss << "\t\t\t\t\t<izz>" << _inertia_vect.z << "</izz>\n";
        sdf_ss << "\t\t\t\t</inertia>\n";
        sdf_ss << "\t\t\t\t<mass>" << _unit_mass << "</mass>\n";
        sdf_ss << "\t\t\t</inertial>\n";

        sdf_ss << "\t\t\t<collision name='" << _model_name << "_collision_" << i << "'>\n";
        sdf_ss << "\t\t\t\t<geometry>\n";
            if(_spawn_geometry == "sphere")
            {
                sdf_ss << "\t\t\t\t\t<sphere>\n";
                sdf_ss << "\t\t\t\t\t\t<radius>" << _unit_size << "</radius>\n";
                sdf_ss << "\t\t\t\t\t</sphere>\n";
            }
            else if(_spawn_geometry == "box")
            {
                sdf_ss << "\t\t\t\t\t<box>\n";
                sdf_ss << "\t\t\t\t\t\t<size>" << _unit_size << " " << _unit_size <<  " " << _unit_size << "</size>\n";
                sdf_ss << "\t\t\t\t\t</box>\n";
            }
            else if(_spawn_geometry == "cylinder")
            {
                sdf_ss << "\t\t\t\t\t<cylinder>\n";
                sdf_ss << "\t\t\t\t\t\t<radius>" << _unit_size << "</radius>\n";
                sdf_ss << "\t\t\t\t\t\t<length>" << _unit_length << "</length>\n";
                sdf_ss << "\t\t\t\t\t</cylinder>\n";
            }
            else if(_spawn_geometry == "mesh")
            {
                sdf_ss << "\t\t\t\t\t<mesh>\n";
                sdf_ss << "\t\t\t\t\t\t<uri>" << _mesh_uri << "</uri>\n";
                sdf_ss << "\t\t\t\t\t\t<scale>" << _mesh_scale << " " << _mesh_scale <<  " " << _mesh_scale << "</scale>\n";
                sdf_ss << "\t\t\t\t\t</mesh>\n";
            }
        sdf_ss << "\t\t\t\t</geometry>\n";


        sdf_ss << "\t\t\t\t<surface>\n";

        // TODO reimplement the sdf parameters as well
        // xml << "\t\t\t\t\t<bounce>\n";
        // xml << "\t\t\t\t\t\t<restitution_coefficient>" << bounce << "</restitution_coefficient>\n";
        // xml << "\t\t\t\t\t\t<threshold>10000.0</threshold>\n";
        // xml << "\t\t\t\t\t</bounce>\n";

        // xml << "\t\t\t\t\t<friction>\n";
        // xml << "\t\t\t\t\t\t<ode>\n";
        // xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
        // xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
        // xml << "\t\t\t\t\t\t\t<fdir1>0.0 0.0 0.0</fdir1>\n";
        // xml << "\t\t\t\t\t\t\t<slip1>" << slip1 << "</slip1>\n";
        // xml << "\t\t\t\t\t\t\t<slip2>" << slip2 << "</slip2>\n";
        // xml << "\t\t\t\t\t\t</ode>\n";
        // xml << "\t\t\t\t\t\t<bullet>\n";
        // xml << "\t\t\t\t\t\t\t<friction>" << friction << "</friction>\n";
        // xml << "\t\t\t\t\t\t\t<friction2>" << friction2 << "</friction2>\n";
        // xml << "\t\t\t\t\t\t\t<rolling_friction>" << roll_friction << "</rolling_friction>\n";
        // xml << "\t\t\t\t\t\t</bullet>\n";
        // xml << "\t\t\t\t\t</friction>\n";

        sdf_ss << "\t\t\t\t\t<contact>\n";
//        sdf_ss << "\t\t\t\t\t\t<ode>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<soft_cfm>" << _cfm << "</soft_cfm>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<soft_erp>" << _erp << "</soft_erp>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<kp>" << _kp << "</kp>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<kd>" << _kd << "</kd>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<max_vel>100.0</max_vel>\n";
//        sdf_ss << "\t\t\t\t\t\t\t<min_depth>0.001</min_depth>\n";
//        sdf_ss << "\t\t\t\t\t\t</ode>\n";
        sdf_ss <<  "\t\t\t\t\t\t<collide_bitmask>" << _collide_bitmask << "</collide_bitmask>\n";
        sdf_ss << "\t\t\t\t\t</contact>\n";

        sdf_ss << "\t\t\t\t</surface>\n";
        sdf_ss << "\t\t\t</collision>\n";

        sdf_ss << "\t\t\t<visual name='" << _model_name << "_visual_" << i << "'>\n";
        sdf_ss << "\t\t\t\t<geometry>\n";
            if(_vis_geometry == "sphere")
            {
                sdf_ss << "\t\t\t\t\t<sphere>\n";
                sdf_ss << "\t\t\t\t\t\t<radius>" << _unit_size << "</radius>\n";
                sdf_ss << "\t\t\t\t\t</sphere>\n";
            }
            else if(_vis_geometry == "box")
            {
                sdf_ss << "\t\t\t\t\t<box>\n";
                sdf_ss << "\t\t\t\t\t\t<size>" << _unit_size << " " << _unit_size <<  " " << _unit_size << "</size>\n";
                sdf_ss << "\t\t\t\t\t</box>\n";
            }
            else if(_vis_geometry == "cylinder")
            {
                sdf_ss << "\t\t\t\t\t<cylinder>\n";
                sdf_ss << "\t\t\t\t\t\t<radius>" << _unit_size << "</radius>\n";
                sdf_ss << "\t\t\t\t\t\t<length>" << _unit_length << "</length>\n";
                sdf_ss << "\t\t\t\t\t</cylinder>\n";
            }
            else if(_vis_geometry == "mesh")
            {
                sdf_ss << "\t\t\t\t\t<mesh>\n";
                sdf_ss << "\t\t\t\t\t\t<uri>" << _mesh_uri << "</uri>\n";
                sdf_ss << "\t\t\t\t\t\t<scale>" << _mesh_scale << " " << _mesh_scale <<  " " << _mesh_scale << "</scale>\n";
                sdf_ss << "\t\t\t\t\t</mesh>\n";
            }
        sdf_ss << "\t\t\t\t</geometry>\n";

        if(_scripts_uri != "")
        {
            sdf_ss << "\t\t\t\t<material>\n";
            sdf_ss << "\t\t\t\t\t<script>\n";
            sdf_ss << "\t\t\t\t\t\t<uri>" << _scripts_uri << "</uri>\n";
            if(_textures_uri != "")
            {
                sdf_ss << "\t\t\t\t\t\t<uri>" << _textures_uri << "</uri>\n";
            }
            sdf_ss << "\t\t\t\t\t\t<name>" << _script_name << "</name>\n";
            sdf_ss << "\t\t\t\t\t</script>\n";
            sdf_ss << "\t\t\t\t</material>\n";
        }

        sdf_ss << "\t\t\t</visual>\n";
        sdf_ss << "\t\t</link>\n";
    }

    sdf_ss << "<allow_auto_disable>false</allow_auto_disable>\n";
    sdf_ss << "<static>" << _static_flag << "</static>\n";
    sdf_ss << "</model>\n";
    sdf_ss << "</sdf>\n";

//    std::cout << sdf_ss.str() << "\n";

    sdf::SDF unitsSDF;
    unitsSDF.SetFromString(sdf_ss.str());
    
    return unitsSDF;    
}


//////////////////////////////////////////////////
math::Vector3 SpawnUnits::ArrangeUnits(
        const int _curr_sphere_index,
        const double _sphere_radius,
        const double _spawn_diameter,
        int& _spawned,
        int& level)
{
    math::Vector3 v3;
    int ii, index_c, c_crt, max_c_in_c;
    double size, R;
    ii = _curr_sphere_index - _spawned;
    size = _sphere_radius * 2;

    v3.z = level * size;
    if (ii != 0)
    {
        index_c = ii-1;
        c_crt = 1;

        while (index_c >= (6*c_crt))
        {
            index_c -= 6*c_crt;
            c_crt++;
        }
        max_c_in_c = c_crt * 6;
        R = c_crt * size;

        if((index_c == (max_c_in_c - 1)) && ((2*R) + (size) >= _spawn_diameter))
        {
            _spawned = _curr_sphere_index+1;
            level++;
        }

        v3.x = R * cos((double) index_c * 2 * PI / max_c_in_c);
        v3.y = R * sin((double) index_c * 2 * PI / max_c_in_c);
    }

    return v3;
}
