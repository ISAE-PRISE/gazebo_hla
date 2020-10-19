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

#ifndef __SHARED_STRUCT_TYPES_H__
#define __SHARED_STRUCT_TYPES_H__

#ifdef __cplusplus
extern "C" 
{
#endif

// ---------------------------------------------------------------------------------
// Actually it has been taken and adapted from Flightgear FDM (needs to be updated)
typedef struct meas_fdm 
{ 
    // Positions
    double longitude;		// geodetic (radians)
    double latitude;		// geodetic (radians)
    double altitude_sl;		// above sea level (meters)
    double altitude_gl;		// above ground level (meters)
    
    double phi;				// roll (radians)
    double theta;			// pitch (radians)
    double psi;				// yaw or true heading (radians)
    
    double alpha;           // angle of attack (radians)
    double beta;            // side slip angle (radians)

    // Velocities
    double phidot;			// roll rate (radians/sec)
    double thetadot;		// pitch rate (radians/sec)
    double psidot;			// yaw rate (radians/sec)
    
	double v_body_u;    	// ECEF velocity in body frame
    double v_body_v;    	// ECEF velocity in body frame 
    double v_body_w;    	// ECEF velocity in body frame
    
    double vcas;		    // calibrated airspeed
    double vtas;		    // true airspeed
    double vias;		    // indicated airspeed
    double veas;		    // equivalent airspeed
    double climb_rate;		// feet per second
    double ground_speed;	// feet per second
    double vmach;
    double v_north;         // north velocity in local/body frame, fps
    double v_east;          // east velocity in local/body frame, fps
    double v_down;          // down/vertical velocity in local/body frame, fps
    double dyn_pressure;

    // Accelerations
    double A_X_pilot;		// X accel in body frame ft/sec^2
    double A_Y_pilot;		// Y accel in body frame ft/sec^2
    double A_Z_pilot;		// Z accel in body frame ft/sec^2
} 
meas_fdm_t;

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif //__SHARED_STRUCT_TYPES_H__
