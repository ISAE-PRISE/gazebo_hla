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

// ----------------------------------------------------------------------------
// GazeboFederateHla13 Constructor
GazeboFederateHla13::GazeboFederateHla13( std::wstring FederationName
											  , std::wstring FederateName
											  , std::wstring FomFileName
											  ) 
											  : NullFederateAmbassador()
											  , _RtiAmb() 
{
	_FederationName = FederationName;
	_FederateName = FederateName;
	_FomFileName = FomFileName;
	_TimeStep = 1.0;
	_Lookahead = 0.001;
	_UavTime = 0.1;
	_SiTime = 0.1;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 1000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = false ;
	_SyncRegSuccess = _SyncRegFailed = _InPause = _IsCreator = false ;
	_IsOutMesTimespamped = true;
	_TimeStampCycles	= NULL;
	memset(&_FdmToSent,0,sizeof(meas_fdm_t));
}

// ----------------------------------------------------------------------------
// GazeboFederateHla13 Destructor
GazeboFederateHla13::~GazeboFederateHla13()
	                     throw(RTI::FederateInternalError)
{
	delete [] _TimeStampCycles;
}

// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
void GazeboFederateHla13::createFederationExecution() 
{
	// To do: Include this in debug flag
	//std::cout << "GazeboFederateHla13.cc -> createFederationExecution(): Start" << std::endl;
    try 
    {
        _RtiAmb.createFederationExecution( getString(_FederationName).c_str()
										 , getString(_FomFileName).c_str()
										 );
		_IsCreator = true;
    } 
    catch ( RTI::FederationExecutionAlreadyExists ) 
    {
		std::cout << "CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
    //std::cout << "GazeboFederateHla13.cc -> createFederationExecution(): End" << std::endl;
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
void GazeboFederateHla13::destroyFederationExecution() 
{
	//std::cout << "GazeboFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb.destroyFederationExecution(getString(_FederationName).c_str());
		}
		catch (RTI::Exception& e) 
		{
			std::cout << "DFE: caught " << e._name << " reason " << e._reason <<std::endl;
		}
	}
	//std::cout << "GazeboFederateHla13.cc -> destroyFederationExecution(): Start" << std::endl;
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::joinFederationExecution() 
{
	//std::cout << "GazeboFederateHla13.cc -> joinFederationExecution(): Start" << std::endl;
	//std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
    try 
    {
        _FederateHandle = _RtiAmb.joinFederationExecution( getString(_FederateName).c_str()
                                                         , getString(_FederationName).c_str()
                                                         ,this
                                                         );
    }
    catch (RTI::Exception& e) 
    {
        std::cout << "JFE: caught " << e._name << " reason " << e._reason <<std::endl;
    }
	//std::cout << "GazeboFederateHla13.cc -> joinFederationExecution(): End" << std::endl;
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::resignFederationExecution() 
{
    //std::cout << "GazeboFederateHla13.cc -> resignFederationExecution(): Start" << std::endl;
    try 
    {
		_RtiAmb.resignFederationExecution(RTI::DELETE_OBJECTS_AND_RELEASE_ATTRIBUTES);
	}
	catch (RTI::Exception& e) 
	{
		std::cout << "RFE: caught " << e._name << " reason " << e._reason <<std::endl;
	}
	//std::cout << "GazeboFederateHla13.cc -> resignFederationExecution(): End" << std::endl;
}

// ----------------------------------------------------------------------------
// Get the federate handle
RTI::FederateHandle GazeboFederateHla13::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
void GazeboFederateHla13::getAllObjectsInteractionsHandles()
{
	//std::cout << "GazeboFederateHla13::getAllObjectsInteractionsHandles: Start" << std::endl;
	// Objects Class
	_ClassHandle = _RtiAmb.getObjectClassHandle("BaseEntity");
	_AttrWorldLocation = _RtiAmb.getAttributeHandle("WorldLocation", _ClassHandle);
	_AttrOrientation = _RtiAmb.getAttributeHandle("Orientation", _ClassHandle);
	_AttrLinearVelocity = _RtiAmb.getAttributeHandle("VelocityVector", _ClassHandle);
	_AttrLinearAcceleration = _RtiAmb.getAttributeHandle("AccelerationVector", _ClassHandle);
	_AttrAngularVelocity = _RtiAmb.getAttributeHandle("AngularVelocityVector", _ClassHandle);
	_AttrVisualModel = _RtiAmb.getAttributeHandle("VisualModel", _ClassHandle);
	_AttrLiveries = _RtiAmb.getAttributeHandle("Liveries", _ClassHandle);
	_AttrFuel= _RtiAmb.getAttributeHandle("Fuel", _ClassHandle);
	
	// Interaction Class
	_InteractionHandle = _RtiAmb.getInteractionClassHandle("VolFormation");
    _ParamRefLon = _RtiAmb.getParameterHandle("RefLon", _InteractionHandle);
	_ParamRefLat = _RtiAmb.getParameterHandle("RefLat", _InteractionHandle);
	_ParamRefAlt = _RtiAmb.getParameterHandle("RefAlt", _InteractionHandle);
	_ParamRefDst = _RtiAmb.getParameterHandle("RefDst", _InteractionHandle);
	_ParamRefVit = _RtiAmb.getParameterHandle("RefVit", _InteractionHandle);	 
	_ParamRefCap = _RtiAmb.getParameterHandle("RefCap", _InteractionHandle);
    _ParamRefForm = _RtiAmb.getParameterHandle("RefForm", _InteractionHandle);
    //std::cout << "GazeboFederateHla13::getAllObjectsInteractionsHandles: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
void GazeboFederateHla13::publishAndSubscribe()
{
	//std::cout << "GazeboFederateHla13::publishAndSubscribe: Start" << std::endl;
    // Get all class and attributes handles
    getAllObjectsInteractionsHandles();

	// Publish and subscribe to object classes
    _AttributeHandleSet.reset(RTI::AttributeHandleSetFactory::create(8));
    _AttributeHandleSet->add(_AttrWorldLocation);
    _AttributeHandleSet->add(_AttrOrientation);
    _AttributeHandleSet->add(_AttrLinearVelocity);
    _AttributeHandleSet->add(_AttrLinearAcceleration);
    _AttributeHandleSet->add(_AttrAngularVelocity);
    _AttributeHandleSet->add(_AttrVisualModel);
    _AttributeHandleSet->add(_AttrLiveries);
    _AttributeHandleSet->add(_AttrFuel);
    _RtiAmb.subscribeObjectClassAttributes( _ClassHandle
                                          , *_AttributeHandleSet
                                          , RTI::RTI_TRUE
                                          );
	_RtiAmb.publishObjectClass( _ClassHandle
                              , *_AttributeHandleSet
                              );   
	_RtiAmb.subscribeInteractionClass(_InteractionHandle, RTI::RTI_TRUE);
    //std::cout << "GazeboFederateHla13::publishAndSubscribe: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
void GazeboFederateHla13::unpublishAndUnsubscribe()
{
	//std::cout << "GazeboFederateHla13::unpublishAndUnsubscribe: Start" << std::endl;
    try 
    {
        _RtiAmb.unpublishObjectClass(_ClassHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        std::cerr << "RTI exception: " << e._name << " ["
        << (e._reason ? e._reason : "undefined") << "]." << std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    try 
    {
        _RtiAmb.unsubscribeObjectClass(_ClassHandle);
        _RtiAmb.unsubscribeInteractionClass(_InteractionHandle);
    } 
    catch ( RTI::Exception &e ) 
    {
        std::cerr << "RTI exception: " << e._name << " ["
        << (e._reason ? e._reason : "undefined") << "]." << std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    //std::cout << "GazeboFederateHla13::unpublishAndUnsubscribe: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Register an Object instance
RTI::ObjectHandle GazeboFederateHla13::registerObjectInstance(std::wstring ObjectName
                                                                )
{
	_MyObjectHandle = _RtiAmb.registerObjectInstance( _ClassHandle
                                         , getString(ObjectName).c_str()
                                         );
    return _MyObjectHandle;
}

// ----------------------------------------------------------------------------
// Register an Object instance
// To do: _RtiAmb.deleteObjectInstance

// ----------------------------------------------------------------------------
// Callback : discover object instance
void GazeboFederateHla13::discoverObjectInstance( RTI::ObjectHandle theObject
                                                   , RTI::ObjectClassHandle theObjectClass
                                                   , const char *theObjectName
                                                   )
                                             throw ( RTI::CouldNotDiscover
                                                   , RTI::ObjectClassNotKnown
                                                   , RTI::FederateInternalError
                                                   )
{
    if (theObjectClass == _ClassHandle)
    {
		bool already = false ;
		for (std::vector<RTI::ObjectHandle>::iterator i = _ObjectHandleVector.begin(); 
			 i != _ObjectHandleVector.end(); 
			 ++i) 
		{
			if (*i == theObject) 
			{
				std::cout << "DOI: Rediscovered object " << theObject << std::endl ;
				already = true ;
			}
		}
		if (!already) 
		{
			meas_fdm_t tmp_fdm;
			memset(&tmp_fdm,0,sizeof(meas_fdm_t));
			_ObjectHandleVector.push_back(theObject);
			_FdmToReceive.insert(TOjtFdmPair(theObject,tmp_fdm));
			std::cout << "DOI: Object discovered" << theObjectName << std::endl;
		}		
	}
    else
    {
		std::string msg = stringize() << "Unknown objectClass < "<<theObjectClass <<">";
        throw RTI::FederateInternalError(msg.c_str());
    }
}

// ----------------------------------------------------------------------------
// Put the federation in pause
void GazeboFederateHla13::pause()
{
    if (_IsCreator) 
    {
		std::cout << ">> CREATOR: PRESS START WHEN ALL FEDERATES ARE READY " << std::endl;      
	    std::getchar();
        std::cout << "Pause requested per Creator " << std::endl;
        try 
        {
            _RtiAmb.registerFederationSynchronizationPoint("Init", "");
        }
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
		while (!_SyncRegSuccess && !_SyncRegFailed) 
		{
			try 
			{
				_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
			} 
			catch ( RTI::Exception &e ) 
			{
				std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
			} 
			catch ( ... ) 
			{
				std::cerr << "Error: unknown non-RTI exception." << std::endl;
			}
			std::cout << ">> Waiting for success or failure of synchronisation point init. " << std::endl;
		} 
		if(_SyncRegFailed)
		{
			std::cout << ">> Error on initial Synchronization" << std::endl;
		} 
    }
	while (!_InPause) 
	{
		std::cout << ">> Waiting for synchronisation point Init announcement." << std::endl;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
	try 
	{
		_RtiAmb.synchronizationPointAchieved("Init");
	} 
	catch ( RTI::Exception &e ) 
	{
		std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
	} 
	catch ( ... ) 
	{
		std::cerr << "Error: unknown non-RTI exception." << std::endl;
	}
	std::cout << ">> Init Synchronisation point satisfied." << std::endl;     

	while (_InPause) 
	{
		std::cout << ">> Waiting for initialization phase." << std::endl ;
		try 
		{
			_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		} 
		catch ( RTI::Exception &e ) 
		{
			std::cerr << "RTI exception: " << e._name << " [" << (e._reason ? e._reason : "undefined") << "]." << std::endl;
		} 
		catch ( ... ) 
		{
			std::cerr << "Error: unknown non-RTI exception." << std::endl;
		}
	} 
}

// ----------------------------------------------------------------------------
// Send Updates
void GazeboFederateHla13::sendUpdate(meas_fdm_t FdmToSent)
{   
	//std::cout << "GazeboFederateHla13::sendUpdateR: Start" << std::endl;
	_FdmToSent = FdmToSent;
    _AttributeHandleValuePairSet.reset(RTI::AttributeSetFactory::create(2));
    _OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_FdmToSent.longitude);
    _OutputMessagebuffer.write_double(_FdmToSent.latitude);
    _OutputMessagebuffer.write_double(_FdmToSent.altitude_sl);
    _OutputMessagebuffer.updateReservedBytes();
     _AttributeHandleValuePairSet -> add(_AttrWorldLocation
										, static_cast<char*>(_OutputMessagebuffer(0))
										, _OutputMessagebuffer.size()
										); 
	_OutputMessagebuffer.reset() ;
    _OutputMessagebuffer.write_double(_FdmToSent.phi);
    _OutputMessagebuffer.write_double(_FdmToSent.theta);
    _OutputMessagebuffer.write_double(_FdmToSent.psi);
    _OutputMessagebuffer.updateReservedBytes();
     _AttributeHandleValuePairSet -> add(_AttrOrientation
										, static_cast<char*>(_OutputMessagebuffer(0))
										, _OutputMessagebuffer.size()
										); 

    try 
    {
        if ( !_IsOutMesTimespamped )
        {
            _RtiAmb.updateAttributeValues(_MyObjectHandle, *_AttributeHandleValuePairSet, "");
        }
        else
        {
            _RtiAmb.updateAttributeValues(_MyObjectHandle, *_AttributeHandleValuePairSet, _LocalTime+_Lookahead, "");
        }
    }
    catch (RTI::Exception& e) 
    {
        std::cout<<"Exception "<<e._name<<" ("<<e._reason<<")"<<std::endl;
    }
	//std::cout << "GazeboFederateHla13::sendUpdateR: End" << std::endl;
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values with time
void GazeboFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const RTI::FedTime& /*theTime*/
                                                   , const char * theTag
                                                   , RTI::EventRetractionHandle /*theHandle*/
                                                   )
											 throw ( RTI::ObjectNotKnown
											       , RTI::AttributeNotKnown
											       , RTI::FederateOwnsAttributes
											       , RTI::InvalidFederationTime 
											       , RTI::FederateInternalError
											       ) 
{
	// Same function as without Logical Time
	reflectAttributeValues(theObject, theAttributes, theTag);
}

// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
void GazeboFederateHla13::reflectAttributeValues( RTI::ObjectHandle theObject
                                                   , const RTI::AttributeHandleValuePairSet& theAttributes
                                                   , const char */*theTag*/
                                                   )
                                             throw ( RTI::ObjectNotKnown
							                       , RTI::AttributeNotKnown
							                       , RTI::FederateOwnsAttributes
							                       , RTI::FederateInternalError
							                       ) 
{
 
	RTI::ULong length ;
	RTI::AttributeHandle handle ;
	MessageBuffer buffer;
	meas_fdm_t tmp_fdm;
	memset(&tmp_fdm,0,sizeof(meas_fdm_t));
	for (std::vector<RTI::ObjectHandle>::iterator i = _ObjectHandleVector.begin(); 
				 i != _ObjectHandleVector.end(); 
				 ++i) 
	{
		if  (*i == theObject)
		{
			for (unsigned int i=0; i<theAttributes.size(); i++) 
			{
				handle = theAttributes.getHandle(i);
				length = theAttributes.getValueLength(i);
				assert(length>0);
				buffer.resize(length);        
				buffer.reset();        
				theAttributes.getValue(i, static_cast<char*>(buffer(0)), length);        
				buffer.assumeSizeFromReservedBytes();

				if (handle == _AttrWorldLocation) 
				{
					
					 tmp_fdm.longitude = buffer.read_double() ; 
					 tmp_fdm.latitude = buffer.read_double() ; 
					 tmp_fdm.altitude_sl = buffer.read_double() ; 
					 std::cout << "GazeboFederateHla13::RAV: theObject"<< theObject << std::endl;
					 _FdmToReceive[theObject] = tmp_fdm;
					 std::cout << "GazeboFederateHla13::RAV: lat"<< _FdmToReceive[theObject].latitude << std::endl;

				} 
				else if (handle == _AttrOrientation) 
				{
					 tmp_fdm.phi = buffer.read_double() ; 
					 tmp_fdm.theta = buffer.read_double() ; 
					 tmp_fdm.psi = buffer.read_double() ; 

				}
				else if (handle == _AttrLinearVelocity) 
				{


				}
				else if (handle == _AttrLinearAcceleration) 
				{


				}
				else if (handle == _AttrAngularVelocity) 
				{


				}
				else if (handle == _AttrVisualModel) 
				{


				}
				else if (handle == _AttrLiveries) 
				{


				}
				else if (handle == _AttrFuel) 
				{


				}
				else    
				{ 
					std::cout << "RAV: Warning, Unknow attribute handle for Object Instance "
					          << theObject 
					          << std::endl; 
				}
			 } 
		} 
	}
}


// ----------------------------------------------------------------------------
// Callback : receive interaction with time
void GazeboFederateHla13::receiveInteraction( RTI::InteractionClassHandle theInteraction
                                               , const RTI::ParameterHandleValuePairSet& theParameters
                                               , const RTI::FedTime& 
                                               , const char *
                                               , RTI::EventRetractionHandle 
                                               )
                                         throw ( RTI::InteractionClassNotKnown
                                               , RTI::InteractionParameterNotKnown
                                               , RTI::InvalidFederationTime
                                               , RTI::FederateInternalError
                                               )
{
	MessageBuffer buffer;
    RTI::ULong valueLength ;

    if (theInteraction == _InteractionHandle) 
    {
		for (unsigned int j = 0 ; j < theParameters.size(); ++j) 
		{
			RTI::ParameterHandle parmHandle = theParameters.getHandle(j);

			valueLength = theParameters.getValueLength(j);
			assert(valueLength>0);
			buffer.resize(valueLength);
			buffer.reset();
			theParameters.getValue(j, static_cast<char*>(buffer(0)), valueLength);        
			buffer.assumeSizeFromReservedBytes();

			if (parmHandle == _ParamRefLat) 
			{                         
			} 
			else    
			{ 
				std::cout << "RAV: Warning, Unknow parameter handle for Interaction "
					          << _InteractionHandle 
					          << std::endl;
			} 
		}
	}
} 

// ----------------------------------------------------------------------------
// Callback : timeRegulationEnabled
void GazeboFederateHla13::timeRegulationEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeRegulationWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeReg = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeConstrainedEnabled
void GazeboFederateHla13::timeConstrainedEnabled(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::EnableTimeConstrainedWasNotPending,
            RTI::FederateInternalError) 
{
	_IsTimeConst = true ;
} 

// ----------------------------------------------------------------------------
// Callback : timeAdvanceGrant
void GazeboFederateHla13::timeAdvanceGrant(const RTI::FedTime& theTime)
    throw ( RTI::InvalidFederationTime,
            RTI::TimeAdvanceWasNotInProgress,
            RTI::FederateInternalError) 
{
	_LocalTime = theTime; 
	_IsTimeAdvanceGrant =  true ;
	// To do include this in a debug flag
	//std::cout << " >> TAG RECU == LocalTime = " <<  _LocalTime << " <<" << std::endl;
} 

// ----------------------------------------------------------------------------
void GazeboFederateHla13::enableTimeRegulation()
{
	_RtiAmb.enableTimeRegulation(_LocalTime, _Lookahead);
	while (!_IsTimeReg) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::disableTimeRegulation()
{
	_RtiAmb.disableTimeRegulation();
	_IsTimeReg = false;
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::enableTimeConstrained()
{
	_RtiAmb.enableTimeConstrained();
	while (!_IsTimeConst) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
	}
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::disableTimeConstrained()
{
	_RtiAmb.disableTimeConstrained();
	_IsTimeConst = false;

}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::enableAsynchronousDelivery()
{
	// enableAsynchronousDelivery() is used to get callback while regulator and constraint
	_RtiAmb.enableAsynchronousDelivery();

}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::disableAsynchronousDelivery()
{
	_RtiAmb.disableAsynchronousDelivery();
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::timeAdvanceRequest()
{

	_RtiAmb.timeAdvanceRequest(_LocalTime + _TimeStep);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::timeAdvanceRequestAvailable(RTIfedTime NextLogicalTime)
{

	_RtiAmb.timeAdvanceRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
void GazeboFederateHla13::nextEventRequest(RTIfedTime NextLogicalTime)
{

	_RtiAmb.nextEventRequest(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// function : nextEventAvailable
void GazeboFederateHla13::nextEventAvailable(RTIfedTime NextLogicalTime)
{

	_RtiAmb.nextEventRequestAvailable(_LocalTime + NextLogicalTime);
	while (!_IsTimeAdvanceGrant) 
	{
		// To do: Is tick2 the correct option...
		_RtiAmb.tick(std::numeric_limits<double>::infinity(), 0.0);
		sched_yield();
	}

	_IsTimeAdvanceGrant = false;
}

// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationSucceeded
void GazeboFederateHla13::synchronizationPointRegistrationSucceeded(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegSuccess = true ;
} 
// ----------------------------------------------------------------------------
// Callback : synchronizationPointRegistrationFailed
void GazeboFederateHla13::synchronizationPointRegistrationFailed(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _SyncRegFailed = true ;
} 

// ----------------------------------------------------------------------------
// Callback : announceSynchronizationPoint
void GazeboFederateHla13::announceSynchronizationPoint(const char *label, const char *tag)
    throw ( RTI::FederateInternalError) 
{
       _InPause = true ;
} 

// ----------------------------------------------------------------------------
// Callback : federationSynchronized
void GazeboFederateHla13::federationSynchronized(const char *label)
    throw ( RTI::FederateInternalError) 
{
    _InPause = false ;
} 

// ----------------------------------------------------------------------------
// function : run
void GazeboFederateHla13::run()
{
	clock_gettime(CLOCK_MONOTONIC, &_TimeStampBegin);
	while (_LocalTime < _SimulationEndTime)
	{
		//sendUpdate(_LocalTime + _Lookahead.getTime());
		//nextEventRequest(_SimulationEndTime);
		//timeAdvanceRequestAvailable(_TimeStep);
	}
	clock_gettime(CLOCK_MONOTONIC, &_TimeStampEnd);
	  
    if ((_TimeStampEnd.tv_nsec-_TimeStampBegin.tv_nsec)<0) {  
        _ExecutionTime.tv_sec = _TimeStampEnd.tv_sec-_TimeStampBegin.tv_sec-1;  
        _ExecutionTime.tv_nsec = 1000000000+_TimeStampEnd.tv_nsec-_TimeStampBegin.tv_nsec;  
    } else {  
        _ExecutionTime.tv_sec = _TimeStampEnd.tv_sec-_TimeStampBegin.tv_sec;  
        _ExecutionTime.tv_nsec = _TimeStampEnd.tv_nsec-_TimeStampBegin.tv_nsec;  
    }  
    
    std::cout << "Execution time :" 
              << _ExecutionTime.tv_sec 
              << " seconds and " 
              << ((_ExecutionTime.tv_nsec) / 1000000)
              << " milliseconds" 
              << std::endl;
	
} 

// ----------------------------------------------------------------------------
// function : open the csv file for the log and variables to store measurments
void GazeboFederateHla13::prepareSimulationLog()
{
	double tm_vector_size;
	if (_IsCreator)
	{
		tm_measurements.open(_OutputFileName);
		tm_measurements.clear();
	}
	// The size is not fixed if we are using NER
	// TO DO: How to define the size of this vector
	// tm_vector_size = static_cast<double> (_SimulationEndTime / _Lookahead);
	//time_cycles = new timespec[tm_vector_size];
	
}

void GazeboFederateHla13::writeSimulationLog()
{	
	tm_measurements  << "Execution time :" 
                     << _ExecutionTime.tv_sec 
                     << " seconds and " 
                     << ((_ExecutionTime.tv_nsec) / 1000000)
                     << " milliseconds" 
                     << std::endl;
	
}

// ----------------------------------------------------------------------------
// function : close the csv file and destroy measurments
void GazeboFederateHla13::closeSimulationLog()
{
	if (_IsCreator)
	{
		tm_measurements.close();
	}
}

// ----------------------------------------------------------------------------
// function : set Lookahead
void GazeboFederateHla13::setLookahead(double lookahead)
{ 
	_Lookahead = lookahead;
	
}

// ----------------------------------------------------------------------------
// function : set TimeStep
void GazeboFederateHla13::setTimeStep(double timestep)
{ 
	_TimeStep = timestep;
}

// ----------------------------------------------------------------------------
// function : se tOutputFileName
void GazeboFederateHla13::setOutputFileName(std::string name)
{ 
	
	_OutputFileName = name;
}

// ----------------------------------------------------------------------------
// function : set localTime
void GazeboFederateHla13::setLocalTime(double localtime)
{
	_LocalTime = localtime;
}

// ----------------------------------------------------------------------------
// function : set SilulationEndtime
void GazeboFederateHla13::setSimulationEndTime(double endtime)
{
	_SimulationEndTime = endtime;
}

// ----------------------------------------------------------------------------
// function : set SilulationEndtime
std::map<RTI::ObjectHandle, meas_fdm_t> GazeboFederateHla13::returnObjectFdmMap()
{
	return _FdmToReceive;
}

meas_fdm_t GazeboFederateHla13::returnObjectFdm(RTI::ObjectHandle theObject)
{
	return _FdmToReceive[theObject];
}
