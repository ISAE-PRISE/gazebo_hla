// ----------------------------------------------------------------------------
// Gazebo HLA - HLA Compliant Gazebo for Run-Time Infrastructure (RTI)
// Copyright (C) 2020  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------

#ifndef Gazebo_MODEL_PLUGIN_HLA13_HH_DEF
#define Gazebo_MODEL_PLUGIN_HLA13_HH_DEF

#include <functional>
#include <iostream>
#include <unistd.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/program_options.hpp>
#include <stdint.h>

// Local 
#include "GazeboFederateHla13.hh"
#include "SharedStructsTypes.h"

namespace gazebo
{
  class GazeboModelPluginHla13 : public ModelPlugin
  {
      
    public: 
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
		// Called by the world update start event
		void OnUpdate();
		
    // Pointer to the model
    private: 
    		// Gazebo Model
		typedef std::pair<RTI::ObjectHandle, physics::ModelPtr> TOjtModelPair;
		typedef std::map<RTI::ObjectHandle, physics::ModelPtr> TOjtModelMap;
		TOjtModelMap _RemoteModel;
		physics::ModelPtr _LocalModel;
		physics::WorldPtr _World;
		long _SimulationStep;
    
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		
		// This is duplicated data from plugin -- Not the best option 
		typedef std::pair<RTI::ObjectHandle, meas_fdm_t> TOjtFdmPair;
		typedef std::map<RTI::ObjectHandle, meas_fdm_t> TOjtFdmMap;
		meas_fdm_t _FdmToSent;
		TOjtFdmMap _FdmToReceive;
		// Pointer to HLA Federate Ambassador
		GazeboFederateHla13 *_MyHla13FederateAmbassador;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginHla13)
}

#endif
