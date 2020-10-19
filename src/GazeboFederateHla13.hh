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

#ifndef Gazebo_FEDERATE_HLA13_HH_DEF
#define Gazebo_FEDERATE_HLA13_HH_DEF

// System includes
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>
#include <iterator>
#include <assert.h>
#include <time.h>
#include <random>
#include <fstream>
#include <limits>
#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// RTI includes
#include <NullFederateAmbassador.hh>
#include <fedtime.hh>
#include <RTI.hh>

// Specific includes
#include "MessageBuffer.hh"
#include "SharedStructsTypes.h"

class GazeboFederateHla13 : public NullFederateAmbassador 
{
	// Private  elements
	private:
	
		// This is duplicated data from plugin -- Not the best option 
		typedef std::pair<RTI::ObjectHandle, meas_fdm_t> TOjtFdmPair;
		typedef std::map<RTI::ObjectHandle, meas_fdm_t> TOjtFdmMap;
		meas_fdm_t _FdmToSent;
		TOjtFdmMap _FdmToReceive;
		
	
		RTI::RTIambassador    _RtiAmb;
	
	    std::wstring          _FederateName;
        std::wstring          _FederationName;
        std::wstring          _FomFileName;
	    
	    std::wstring          _InitSyncPointName;
	    std::string           _OutputFileName;
	    
	    RTI::FederateHandle _FederateHandle ;
	
		bool _IsTimeReg;
		bool _IsTimeConst;
		bool _IsTimeAdvanceGrant;
		bool _IsOutMesTimespamped;

		RTIfedTime _TimeStep;   
		RTIfedTime _Lookahead;
		RTIfedTime _UavTime;   
		RTIfedTime _SiTime;      
		RTIfedTime _RestOfTimeStep; 			
		RTIfedTime _LocalTime;
		RTIfedTime _SimulationEndTime;

		bool _SyncRegSuccess;
		bool _SyncRegFailed;
		bool _InPause;
		bool _IsCreator;
		
		MessageBuffer _OutputMessagebuffer;

		RTI::ObjectClassHandle _ClassHandle;
		RTI::ObjectHandle _MyObjectHandle;
		std::vector<RTI::ObjectHandle> _ObjectHandleVector;
		RTI::AttributeHandle _AttrWorldLocation;
		RTI::AttributeHandle _AttrOrientation;
		RTI::AttributeHandle _AttrLinearVelocity;
		RTI::AttributeHandle _AttrLinearAcceleration;
		RTI::AttributeHandle _AttrAngularVelocity;
		RTI::AttributeHandle _AttrVisualModel;
		RTI::AttributeHandle _AttrLiveries;
		RTI::AttributeHandle _AttrFuel;
		std::unique_ptr<RTI::AttributeHandleSet> _AttributeHandleSet;
		std::unique_ptr<RTI::AttributeHandleValuePairSet> _AttributeHandleValuePairSet;

		RTI::ObjectClassHandle mClassHandle_ControlFed;
		RTI::ObjectHandle mObjectHandleControlFed;
		RTI::AttributeHandle mAttrMoveNow;
		RTI::AttributeHandle mAttrNewFlotillaPosition;
		
		RTI::InteractionClassHandle _InteractionHandle ;
		RTI::ParameterHandle _ParamRefLon;
		RTI::ParameterHandle _ParamRefLat;
		RTI::ParameterHandle _ParamRefAlt;
		RTI::ParameterHandle _ParamRefDst;
		RTI::ParameterHandle _ParamRefVit;	 
		RTI::ParameterHandle _ParamRefCap;
		RTI::ParameterHandle _ParamRefForm;
		std::unique_ptr<RTI::ParameterHandleValuePairSet>  _ParameterHandleValuePairSet;
		
		// The creator will use this to store measurements
		std::ofstream tm_measurements;
		//double *d_cycles;
		timespec _TimeStampBegin, _TimeStampEnd;
		timespec _ExecutionTime;
		timespec _TimeStamp1, _TimeStamp2;
		timespec *_TimeStampCycles;
				
    // Public elements
	public:
	
		GazeboFederateHla13 ( std::wstring FederationName
							   , std::wstring FederateName
					           , std::wstring FomFileName
					           );

		virtual ~GazeboFederateHla13() throw(RTI::FederateInternalError);
		
		// wstring handling
		static const std::string getString(const std::wstring& wstr) {
			return std::string(wstr.begin(),wstr.end());
		};

		static const std::wstring getWString(const char* cstr) {
			std::wstringstream ss;
			ss << cstr;
			return std::wstring(ss.str());
		};
		
        void setLookahead(double lookahead);
        void setTimeStep(double timestep);
        void setOutputFileName(std::string name);
        void setLocalTime(double localtime);
        void setSimulationEndTime(double endtime);
        
        // Gazebo
        void setParentModel(double endtime);
        
		// Federation Management
		void createFederationExecution();
		void destroyFederationExecution();

		void joinFederationExecution();
		void resignFederationExecution();
		
		RTI::FederateHandle getFederateHandle() const ;
		
		void getAllObjectsInteractionsHandles();
		
		void publishAndSubscribe();
		void unpublishAndUnsubscribe();
		
		void run();
		
		RTI::ObjectHandle registerObjectInstance(std::wstring ObjectName);
                                                
		void pause();
		
		
		void receiveInteraction(RTI::InteractionClassHandle theInteraction,
					const RTI::ParameterHandleValuePairSet& theParameters,
					const RTI::FedTime& theTime, const char *theTag,
					RTI::EventRetractionHandle theHandle)
		throw (RTI::InteractionClassNotKnown, RTI::InteractionParameterNotKnown,
		RTI::InvalidFederationTime, RTI::FederateInternalError);

		/* void receiveInteraction(RTI::InteractionClassHandle,
					const RTI::ParameterHandleValuePairSet &,
					const char *)
		throw (RTI::InteractionClassNotKnown, RTI::InteractionParameterNotKnown,
		RTI::FederateInternalError); */
	

		// Callback : discover object instance
		void discoverObjectInstance ( RTI::ObjectHandle theObject
									, RTI::ObjectClassHandle theObjectClass
									, const char *theObjectName
									)
		throw ( RTI::CouldNotDiscover,
			RTI::ObjectClassNotKnown,
			RTI::FederateInternalError);
			
		void sendUpdate(meas_fdm_t FdmToSent);

		// Callback : reflect attribute values without time
		void reflectAttributeValues ( RTI::ObjectHandle theObject
									, const RTI::AttributeHandleValuePairSet& theAttributes
									, const char *theTag
									)
							  throw ( RTI::ObjectNotKnown
							        , RTI::AttributeNotKnown
							        , RTI::FederateOwnsAttributes
							        , RTI::FederateInternalError
							        ) ;

		// Callback : reflect attribute values with time
		void reflectAttributeValues ( RTI::ObjectHandle theObject
									, const RTI::AttributeHandleValuePairSet& theAttributes
									, const RTI::FedTime& /*theTime*/
									, const char *theTag
									, RTI::EventRetractionHandle
									)
							  throw ( RTI::ObjectNotKnown
								    , RTI::AttributeNotKnown
								    , RTI::FederateOwnsAttributes
								    , RTI::InvalidFederationTime 
								    , RTI::FederateInternalError
								    ) ;


		// HLA specific methods : TIME MANAGEMENT 
		// Callback : timeRegulationEnabled
		void timeRegulationEnabled(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::EnableTimeRegulationWasNotPending,
			RTI::FederateInternalError) ;

		// Callback : timeConstrainedEnabled
		void timeConstrainedEnabled(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::EnableTimeConstrainedWasNotPending,
			RTI::FederateInternalError) ;

		// Callback : timeAdvanceGrant
		void timeAdvanceGrant(const RTI::FedTime& theTime)
		throw ( RTI::InvalidFederationTime,
			RTI::TimeAdvanceWasNotInProgress,
			RTI::FederateInternalError) ;
			
		void enableTimeRegulation();
		void enableTimeConstrained();
		void enableAsynchronousDelivery();
		void disableTimeRegulation();
		void disableTimeConstrained();
		void disableAsynchronousDelivery();
		void timeAdvanceRequest();
		void nextEventRequest(RTIfedTime NextLogicalTime);
		void timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime);
		void nextEventAvailable(RTIfedTime NextLogicalTime);

		// HLA specific methods : SYNCHRONISATION 
		// Callback : synchronizationPointRegistrationSucceeded
		void synchronizationPointRegistrationSucceeded(const char *label)
		throw (RTI::FederateInternalError) ;

		// Callback : synchronizationPointRegistrationFailed
		void synchronizationPointRegistrationFailed(const char *label)
		throw (RTI::FederateInternalError) ;

		// Callback : announceSynchronizationPoint
		void announceSynchronizationPoint(const char *label, const char *tag)
		throw (RTI::FederateInternalError) ;

		// Callback : federationSynchronized
		void federationSynchronized(const char *label)
		throw (RTI::FederateInternalError) ;
		
		void prepareSimulationLog();
		void writeSimulationLog();
		void closeSimulationLog();
		
		std::map<RTI::ObjectHandle, meas_fdm_t> returnObjectFdmMap();
		meas_fdm_t returnObjectFdm(RTI::ObjectHandle);
		
};

#endif //Gazebo_FEDERATE_HLA13_HH_DEF
