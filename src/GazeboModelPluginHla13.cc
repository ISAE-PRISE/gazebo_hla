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

#include "GazeboModelPluginHla13.hh"

namespace pt = boost::property_tree;
namespace po = boost::program_options;

void
usage(const po::options_description& desc)
{
    std::cout << "Usage: optitrack_node [options]\n";
    std::cout << desc;
}


namespace gazebo {
	
void GazeboModelPluginHla13::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	printf("[module] Load - begin");

	  /* initialize random seed: */
	  srand (time(NULL));

	  /* generate secret number between 1 and 10: */
	  int id = rand() % 1000 + 1;
	  std::string id_string = std::to_string(id);
	
	int DataSizeByte               =  1000;
	double lookahead               =  1.0;
	double timestep                =  10.0;
	double localtime               =  0.0;
	double endtime                 =  100000.0;
	
	memset(&_FdmToSent,0,sizeof(meas_fdm_t));


	std::string FederationName =  "AnsvaFederation";
	std::string FederateName   =  "Federate_" + id_string;
	std::string FomFile        =  "ASNVA.fed";
	std::wstring federationName = GazeboFederateHla13::getWString(FederationName.c_str());
	std::wstring federateName = GazeboFederateHla13::getWString(FederateName.c_str());
	std::wstring FOMpath = GazeboFederateHla13::getWString(FomFile.c_str());

	_MyHla13FederateAmbassador = new GazeboFederateHla13(federationName, federateName, FOMpath);
	_MyHla13FederateAmbassador->createFederationExecution();
	_MyHla13FederateAmbassador->joinFederationExecution();
	_MyHla13FederateAmbassador->publishAndSubscribe();

	// Set lookahad, timestep, name of time mesurement, localtime and simulationtime.
	_MyHla13FederateAmbassador->setLookahead(lookahead);
	_MyHla13FederateAmbassador->setTimeStep(timestep);
	_MyHla13FederateAmbassador->setLocalTime(localtime);
	_MyHla13FederateAmbassador->setSimulationEndTime(endtime);

	std::wstring Object_Name = L"Object_" + federateName;
	_MyHla13FederateAmbassador->registerObjectInstance(Object_Name);

	_MyHla13FederateAmbassador->enableTimeRegulation();
	_MyHla13FederateAmbassador->enableTimeConstrained();
	_MyHla13FederateAmbassador->enableAsynchronousDelivery();
	_MyHla13FederateAmbassador->pause();
	printf("[module] Load - HLA init OK\n");
	
	// Store the pointer to the model
	this->_LocalModel = _parent;
	this->_World = this->_LocalModel->GetWorld();
	this->_SimulationStep = 0;
	
	printf("[module] Load - Model and World loaded OK\n");
  
	_FdmToReceive = _MyHla13FederateAmbassador->returnObjectFdmMap();
	for( const auto& obj_fdm_pair : _FdmToReceive )
	{
		physics::ModelPtr tmp_model = this->_World->GetModel("box1"); ;
		_RemoteModel.insert(TOjtModelPair(obj_fdm_pair.first,tmp_model));
		this->_LocalModel->AttachStaticModel(tmp_model,math::Pose(0,0,0.6, 0, 0, 0));
	}
	printf("[module] Load - instanciate remote model\n");

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboModelPluginHla13::OnUpdate, this));
	printf("[module] Load - end");
}

void GazeboModelPluginHla13::OnUpdate()
{
	printf("[module] OnUpdate - begin\n");
	//SetWorldPose
	math::Pose WorldPosition;
	math::Pose WorldPositionRcv;
	WorldPosition = this->_LocalModel->GetWorldPose() ;
	_FdmToSent.longitude = WorldPosition.pos.x;
	_FdmToSent.latitude = WorldPosition.pos.y;
	_FdmToSent.altitude_sl = WorldPosition.pos.z;
	meas_fdm_t tmp_fdm;
	memset(&tmp_fdm,0,sizeof(meas_fdm_t));
	
	//_FdmToReceive = _MyHla13FederateAmbassador->returnObjectFdmMap();
	//std::cout<< "WorldPositionRcv.pos.x : " << _FdmToReceive[0].longitude << std::endl;
	for( const auto& obj_mod_pair : _RemoteModel )
	{
		tmp_fdm = _MyHla13FederateAmbassador->returnObjectFdm(obj_mod_pair.first);
		/* WorldPositionRcv.pos.x = tmp_fdm.longitude - WorldPosition.pos.x;
		WorldPositionRcv.pos.y = tmp_fdm.latitude - WorldPosition.pos.y;
		WorldPositionRcv.pos.z = tmp_fdm.altitude_sl -WorldPosition.pos.z;*/
		WorldPositionRcv.pos.x = tmp_fdm.longitude;
		WorldPositionRcv.pos.y = tmp_fdm.latitude ;
		WorldPositionRcv.pos.z = tmp_fdm.altitude_sl;
		obj_mod_pair.second->SetWorldPose(WorldPositionRcv);
		physics::ModelPtr tmp_model = obj_mod_pair.second; 
		this->_LocalModel->AttachStaticModel(tmp_model,WorldPositionRcv);
	}
	
	_MyHla13FederateAmbassador->sendUpdate(_FdmToSent);
	_MyHla13FederateAmbassador->timeAdvanceRequest();

	++_SimulationStep;
	// Apply a small linear velocity to the model.
	this->_LocalModel->SetLinearVel(ignition::math::Vector3d(0.3, 0.1, 0.4));
	std::cout<< "plugin count :" << _SimulationStep << std::endl;
	std::cout<< "WorldPosition.pos.x : " << WorldPosition.pos.x << std::endl;
	std::cout<< "WorldPosition.pos.y : " << WorldPosition.pos.y << std::endl;
	std::cout<< "WorldPosition.pos.Z : " << WorldPosition.pos.z << std::endl;
	std::cout<< "WorldPositionRcv.pos.x : " << WorldPositionRcv.pos.x << std::endl;
	std::cout<< "WorldPositionRcv.pos.y : " << WorldPositionRcv.pos.y << std::endl;
	std::cout<< "WorldPositionRcv.pos.Z : " << WorldPositionRcv.pos.z << std::endl;
	//usleep(100000);


	//node->step();
	//node->maj_Pose(_pose.pos.x, _pose.pos.y, _pose.pos.z);
	printf("[module] OnUpdate - end\n");
}
  
} // end of namespace gazebo
