// ----------------------------------------------------------------------------
// Gazebo HLA - HLA Compliant Gazebo for Run-Time Infrastructure (RTI)
// Copyright (C) 2018  ISAE
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

#include "GazeboFederateHla13.hh"

#include <iostream>
#include <sstream>
#include <cstdlib>

int main(int argc, char *argv[])
{
 	/* Main program inputs */
	std::string FederationName =  argv[1];
	std::string FederateName   =  argv[2];
	std::string FomFile        =  argv[3];
	
	std::string DataSizeString     =  argv[4];
    int DataSizeByte               =  atoi(argv[4]);
	std::string LookaheadString    =  argv[5];
	double lookahead               =  atof(argv[5]);
	std::string TimestepString     =  argv[6];
	double timestep                =  atof(argv[6]); 
	std::string LocalTimeString    =  argv[7];
	double localtime               =  atof(argv[7]);
	std::string EndTimeString      =  argv[8];
	double endtime                 =  atof(argv[8]);
	
	std::string OutputFileName =  FederateName + "_" + DataSizeString + "_" + LookaheadString + "_" + TimestepString + "_" + LocalTimeString + "_" + EndTimeString;
	/* Debug Main Arguments */
    std::cout << "The following arguments were passed to main(): "<< std::endl;
	std::cout << "Federation Name is: "<< FederationName << std::endl;
	std::cout << "Federate Name is: "<< FederateName << std::endl;
	std::cout << "Fom File is: "<< FomFile << std::endl;
	std::cout << "DataSizeByte is: "<< DataSizeString << std::endl;
	std::cout << "Lookahead is: "<< lookahead << std::endl;
	std::cout << "TimeStep is: "<< timestep << std::endl;
	std::cout << "This Federate starts at : "<< localtime << std::endl;
	std::cout << "This Federate ends at : "<< endtime << std::endl;
			
		
  std::wstring federationName = GazeboFederateHla13::getWString(FederationName.c_str());
  std::wstring federateName = GazeboFederateHla13::getWString(FederateName.c_str());
  std::wstring FOMpath = GazeboFederateHla13::getWString(FomFile.c_str());
  
  // Create a federate object.
  // This object inherit from appropriate FederateAmbassador class
  // and embbed the appropriate RTIambassador object.
  GazeboFederateHla13 myFederate(federationName, federateName, FOMpath);
  myFederate.createFederationExecution();
  myFederate.joinFederationExecution();
  myFederate.publishAndSubscribe();
  
  // Allocate buffers for read/write
  myFederate.allocateAndFillDatas(DataSizeByte);
  // Set lookahad, timestep, name of time mesurement, localtime and simulationtime.
  myFederate.setLookahead(lookahead);
  myFederate.setTimeStep(timestep);
  myFederate.setLocalTime(localtime);
  myFederate.setSimulationEndTime(endtime);
 
  std::wstring Object_R_Name = L"Object_R" + federateName;
  RTI::ObjectClassHandle Object_R_ClassHandle;
  Object_R_ClassHandle = myFederate.get_ObjectExampleR_ID();
  myFederate.registerObjectInstance(Object_R_ClassHandle, Object_R_Name);
  
  std::wstring Object_BE_Name = L"Object_BE" + federateName;
  RTI::ObjectClassHandle Object_BE_ClassHandle;
  Object_BE_ClassHandle = myFederate.get_ObjectExampleBE_ID();
  myFederate.registerObjectInstance(Object_BE_ClassHandle, Object_BE_Name);
  myFederate.enableTimeRegulation();
  myFederate.enableTimeConstrained();
  myFederate.enableAsynchronousDelivery();
  myFederate.pause();
  
  myFederate.run();
  
  myFederate.setOutputFileName(OutputFileName);
  myFederate.prepareSimulationLog();
  myFederate.writeSimulationLog();
  myFederate.closeSimulationLog();
  
  myFederate.unpublishAndUnsubscribe();
  myFederate.resignFederationExecution();
  myFederate.destroyFederationExecution();
  return 1;
}

