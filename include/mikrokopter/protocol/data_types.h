/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dirk Holz, University of Bonn.
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
 */
#ifndef KOPTER_PROTOCOL_DATA_TYPES_H_
#define KOPTER_PROTOCOL_DATA_TYPES_H_

#include <string>
#include <stdint.h>

#define DEBUG_PRINT_VARIABLE(A, B) A << #B << " :\t\t\t" << B << std::endl;

namespace mikrokopter
{
  namespace protocol
  {

    const int ADDRESS_BROADCAST = 0;
    const int ADDRESS_FLIGHT_CTRL = 1;
    const int ADDRESS_NAVI_CTRL = 2;
    const int ADDRESS_MK3MAG = 3;
    const int ADDRESS_BL_CTRL = 5;

    const char MSG_SELECT_NAVI_CTRL [] = {0x1B, 0x1B, 0x55, 0xAA, 0x00};

    /**
     * External control data (e.g., for communication over serial line).
     * The idea is to set roll, pitch, and yaw angles,
     * and to set gas to maximum (so that it is controlled from the RC.
     * Height control (seems to be not working) and should be deactivated.
     * \brief FlightCtrl / NaviCtrl / MK3MAG version
     * \note copied from FlightCtrl/uart.h:52
     * \note processed in ClightCtrl/fc.c:950
     */
    struct ExternControl // FLIGHT_CTRL
    {
      ExternControl()
        : digital_0(0), digital_1(0)
        , remote_buttons(0)
        , pitch(0), roll(0), yaw(0)
        , gas(0), height(0)
        , free(0), request_acknowledgement(0)
        , config(0){}
      unsigned char digital_0;
      unsigned char digital_1;
      unsigned char remote_buttons;
      signed char   pitch;
      signed char   roll;
      signed char   yaw;
      unsigned char gas;
      signed char   height;  // height value for height control
      unsigned char free;    // don't know
      unsigned char request_acknowledgement;   // requests a reply in the form of "B"
      unsigned char config;  // 1 for activating control, 0 for disabling it
    } __attribute__((packed));


    struct FlightControlDebugData // FLIGHT_CTRL
    {
      uint8_t status[2];                   // check for ERROR flags
      union
      {
        int16_t data[32];                  // signed int Analog[32];
        struct
        {
          int16_t integrated_nick;         // DebugOut.Analog[0] = IntegralNick / (EE_Parameter.GyroAccFaktor * 4);
          int16_t integrated_roll;         // DebugOut.Analog[1] = IntegralRoll / (EE_Parameter.GyroAccFaktor * 4);
          int16_t mean_acceleration_nick;  // DebugOut.Analog[2] = Mittelwert_AccNick / 4;
          int16_t mean_acceleration_roll;  // DebugOut.Analog[3] = Mittelwert_AccRoll / 4;
          int16_t yaw;                     // DebugOut.Analog[4] = (signed int) AdNeutralGier - AdWertGier;
          int16_t height;                  // DebugOut.Analog[5] = HoehenWert/5;
          int16_t acceleration_up;         // DebugOut.Analog[6] = AdWertAccHoch;//(Mess_Integral_Hoch / 512);// Aktuell_az;
          int16_t gas;                     // [7] !!! FIND OUT WHERE THIS IS SET!!!!!!!
          int16_t compass;                 // DebugOut.Analog[8] = KompassValue;
          int16_t battery_voltage;         // DebugOut.Analog[9] = UBat;
          int16_t sensor_ok;               // DebugOut.Analog[10] = SenderOkay;
          int16_t backup_compass_deg;      // DebugOut.Analog[11] = ErsatzKompassInGrad;
          int16_t motor_set_points[4];     // DebugOut.Analog[12] [13] [14] [15] = Motor[0].SetPoint;
          int16_t stick_nick;              // !!! [16] [17] [18] [19] are commented out or not used at all in the original FLightCtrl sources
          int16_t stick_roll;              // !!! [16] [17] [18] [19] are commented out or not used at all in the original FLightCtrl sources
          int16_t stick_gier;              // !!! [16] [17] [18] [19] are commented out or not used at all in the original FLightCtrl sources
          int16_t stick_gas;               // !!! [16] [17] [18] [19] are commented out or not used at all in the original FLightCtrl sources
          int16_t servo_nick;              // DebugOut.Analog[20] = ServoNickValue; 
          int16_t hover_gas;               // !!! [21] ./fc.c:1396: DebugOut.Analog[21] = HoverGas;
          int16_t current;                 // DebugOut.Analog[22] = Capacity.ActualCurrent;
          int16_t capacity_used;           // DebugOut.Analog[23] = Capacity.UsedCapacity;
          int16_t height_set_point;        // DebugOut.Analog[24] = SollHoehe/5;
          int16_t external_control;        // !!! [25] and [26] are not used at all (25), or commented out (26) in the original FLightCtrl sources
          int16_t another_unused;          // !!! [25] and [26] are not used at all (25), or commented out (26) in the original FLightCtrl sources
          int16_t compass_set_point;       // DebugOut.Analog[27] = KompassSollWert;
          int16_t i2c_error;               // ./main.c:285: DebugOut.Analog[28]++; // I2C-Error
          int16_t capacity_min_of_max_pwm; // DebugOut.Analog[29] = Capacity.MinOfMaxPWM;
          int16_t gps_nick;                // DebugOut.Analog[30] = GPS_Nick;
          int16_t gps_roll;                // DebugOut.Analog[31] = GPS_Roll;
        } fc_debug __attribute__((packed));
      } __attribute__((packed));
    } __attribute__((packed));

    // typedef char FlightControlDebugDataLabels[32][16];
    const unsigned int DEBUG_LABEL_LENGTH = 16;
    typedef std::string FlightControlDebugDataLabels[32];

    /**
     * \brief FlightCtrl / NaviCtrl / MK3MAG version
     * \note copied from FlightCtrl/uart.h:88
     */
    struct VersionInfo
    {
      VersionInfo () : SWMajor (0), SWMinor (0), ProtoMajor (0), ProtoMinor (0), SWPatch (0) {}
      unsigned char SWMajor;
      unsigned char SWMinor;
      unsigned char ProtoMajor;
      unsigned char ProtoMinor;
      unsigned char SWPatch;
      unsigned char HardwareError[5];
    } __attribute__((packed));


    /** from NaviCtrl/ubx.h:24 */
    struct GPS_Pos
    {
      int32_t    Longitude;               // in 1E-7 deg
      int32_t    Latitude;	              // in 1E-7 deg
      int32_t    Altitude;	              // in mm
      uint8_t    Status;                  // validity of data
    } __attribute__((packed));

    /** from NaviCtrl/uart1.h:83 */
    struct GPS_PosDev
    {
      uint16_t   Distance;                // distance to target in dm
      int16_t    Bearing;                 // course to target in deg
    } __attribute__((packed));

    /** from NaviCtrl/uart1.h:91 */
    const unsigned int NAVI_DATA_VERSION = 5;
    struct NaviData
    {
      uint8_t    Version;						      // version of the data structure
      GPS_Pos    CurrentPosition;		      // see ubx.h for details
      GPS_Pos    TargetPosition;
      GPS_PosDev TargetPositionDeviation;
      GPS_Pos    HomePosition;
      GPS_PosDev HomePositionDeviation;
      uint8_t    WaypointIndex;				    // index of current waypoints
      uint8_t    WaypointNumber;				  // number of stored waypoints
      uint8_t    SatsInUse;					      // number of satellites used for position solution
      int16_t    Altimeter; 					    // hight according to air pressure
      int16_t    Variometer;					    // climb(+) and sink(-) rate
      uint16_t   FlyingTime;					    // in seconds
      uint8_t    UBat;						        // Battery Voltage in 0.1 Volts
      uint16_t   GroundSpeed;				      // speed over ground in cm/s (2D)
      int16_t    Heading;					        // current flight direction in ° as angle to north
      int16_t	   CompassHeading;				  // current compass value in °
      int8_t     AngleNick;					      // current Nick angle in 1°
      int8_t     AngleRoll;					      // current Rick angle in 1°
      uint8_t    RC_Quality;					    // RC_Quality
      uint8_t    FCStatusFlags;				    // Flags from FC
      uint8_t    NCFlags;					        // Flags from NC
      uint8_t    Errorcode;					      // 0 --> okay
      uint8_t    OperatingRadius;			    // current operation radius around the Home Position in m
      int16_t    TopSpeed;					      // velocity in vertical direction in cm/s
      uint8_t    TargetHoldTime;				  // time in s to stay at the given target,
      // counts down to 0 if target has been reached
      uint8_t    FCStatusFlags2;				  // StatusFlags2 (since version 5 added)
      int16_t    SetpointAltitude;			  // setpoint for altitude
      uint8_t    Gas;						          // for future use
      uint16_t   Current;					        // actual current in 0.1A steps
      uint16_t   UsedCapacity;				    // used capacity in mAh
    } __attribute__((packed));



  }
}


#endif
