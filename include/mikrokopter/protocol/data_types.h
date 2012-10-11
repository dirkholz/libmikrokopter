/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/* DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *                    Version 2, December 2004
 *
 *  Copyright (c) 2012, Dirk Holz, dirk.holz@ieee.org
 *
 * Everyone is permitted to copy and distribute verbatim or modified
 * copies of this license document, and changing it is allowed as long
 * as the name is changed.
 *
 *            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 *
 *  0. You just DO WHAT THE FUCK YOU WANT TO.
 *
 * This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://sam.zoy.org/wtfpl/COPYING for more details.
 *
 */
#ifndef KOPTER_PROTOCOL_DATA_TYPES_H_
#define KOPTER_PROTOCOL_DATA_TYPES_H_

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

    const char MSG_SELECT_NAVI_CTRL   [] = { 0x1B, 0x1B, 0x55, 0xAA, '\r', '\0' };

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
          : config(0){}
      unsigned char digital[2];
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
          int16_t motor_set_points[4];     // DebugOut.Analog[12] = Motor[0].SetPoint;
          // DebugOut.Analog[13] = Motor[1].SetPoint;
          // DebugOut.Analog[14] = Motor[2].SetPoint;
          // DebugOut.Analog[15] = Motor[3].SetPoint;
          int16_t unused[5];                // !!! [16] [17] [18] [19] are commented out or not used at all in the FLightCtrl sources
          int16_t servo_nick;              // DebugOut.Analog[20] = ServoNickValue;\
          int16_t hover_gas;               // !!! [21] ./fc.c:1396: DebugOut.Analog[21] = HoverGas;
          int16_t current;                 // DebugOut.Analog[22] = Capacity.ActualCurrent;
          int16_t capacity_used;           // DebugOut.Analog[23] = Capacity.UsedCapacity;
          int16_t height_set_point;        // DebugOut.Analog[24] = SollHoehe/5;
          int16_t another_unused[2];       // !!! [25] and [26] are not used at all (25), or commented out (26)
          int16_t compass_set_point;       // DebugOut.Analog[27] = KompassSollWert;
          int16_t i2c_error;               // ./main.c:285: DebugOut.Analog[28]++; // I2C-Error
          int16_t capacity_min_of_max_pwm; // DebugOut.Analog[29] = Capacity.MinOfMaxPWM;
          int16_t gps_nick;                // DebugOut.Analog[30] = GPS_Nick;
          int16_t gps_roll;                // DebugOut.Analog[31] = GPS_Roll;
        } __attribute__((packed));
      } __attribute__((packed));
    } __attribute__((packed));

    // typedef char FlightControlDebugDataLabels[32][16];
    const unsigned int DEBUG_LABEL_LENGTH = 16;
    typedef std::string FlightControlDebugDataLabels[32];

    // std::ostream & operator<<(std::ostream &os, const FlightControlDebugData& p)
    // {
    //   DEBUG_PRINT_VARIABLE(os, p.integrated_nick);
    //   DEBUG_PRINT_VARIABLE(os, p.integrated_roll);
    //   DEBUG_PRINT_VARIABLE(os, p.mean_acceleration_nick);
    //   DEBUG_PRINT_VARIABLE(os, p.mean_acceleration_roll);
    //   DEBUG_PRINT_VARIABLE(os, p.yaw);
    //   DEBUG_PRINT_VARIABLE(os, p.height);
    //   DEBUG_PRINT_VARIABLE(os, p.acceleration_up);
    //   DEBUG_PRINT_VARIABLE(os, p.gas);
    //   DEBUG_PRINT_VARIABLE(os, p.compass);
    //   DEBUG_PRINT_VARIABLE(os, p.battery_voltage);
    //   DEBUG_PRINT_VARIABLE(os, p.sensor_ok);
    //   DEBUG_PRINT_VARIABLE(os, p.backup_compass_deg);
    //   DEBUG_PRINT_VARIABLE(os, p.motor_set_points[0]);
    //   DEBUG_PRINT_VARIABLE(os, p.motor_set_points[1]);
    //   DEBUG_PRINT_VARIABLE(os, p.motor_set_points[2]);
    //   DEBUG_PRINT_VARIABLE(os, p.motor_set_points[3]);
    //   DEBUG_PRINT_VARIABLE(os, p.servo_nick);
    //   DEBUG_PRINT_VARIABLE(os, p.current);
    //   DEBUG_PRINT_VARIABLE(os, p.capacity_used);
    //   DEBUG_PRINT_VARIABLE(os, p.height_set_point);
    //   DEBUG_PRINT_VARIABLE(os, p.compass_set_point);
    //   DEBUG_PRINT_VARIABLE(os, p.capacity_min_of_max_pwm);
    //   DEBUG_PRINT_VARIABLE(os, p.gps_nick);
    //   DEBUG_PRINT_VARIABLE(os, p.gps_roll);
    // }

    /**
     * \brief FlightCtrl / NaviCtrl / MK3MAG version
     * \note copied from FlightCtrl/uart.h:88
     */
    struct VersionInfo
    {
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