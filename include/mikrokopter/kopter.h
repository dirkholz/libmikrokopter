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
#ifndef KOPTER_KOPTER_H_
#define KOPTER_KOPTER_H_

#include <boost/thread.hpp>
#include <mikrokopter/common/running_average.h>
#include <mikrokopter/common/time.h>
#include <mikrokopter/io/io.h>
#include <mikrokopter/protocol/protocol.h>

namespace mikrokopter
{
  class Kopter
  {
    typedef boost::function<void(const mikrokopter::protocol::FlightControlDebugData&)> FlightControlDebugDataCallbackType;
    
    public:
      Kopter(mikrokopter::io::IO::Ptr& comm);
    
      bool connectNaviCtrl();    
    
      bool connectFlightCtrl();
    
      bool connectMK3MAG();
    
      void getVersionInfoBlocking();
    
      void parseMessage(const std::string& message);
    
      inline mikrokopter::io::IO::Ptr getComm() { return comm_; }
    
      void sendExternalControl(const mikrokopter::protocol::ExternControl& control);
    
      /**
       * request debug data from FlightCtrl
       */
      void requestDebugData(int interval);

      void requestDebugDataLabel(const uint8_t& id);

      /**
       * request debug data from NaviCtrl
       */
      void requestNaviData(int interval);
    
      void requestDebugDataLabels();

      inline void printFlightControlDebugData()
      {
        printFlightControlDebugData(flight_control_debug_data_,
                                    flight_control_debug_data_labels_);
      }

      inline mikrokopter::protocol::FlightControlDebugData getDebugData()
      {
        return flight_control_debug_data_;
      }
    
      void registerFlightControlCallback(FlightControlDebugDataCallbackType cb);

    
    protected:

      mikrokopter::io::IO::Ptr comm_;
      mikrokopter::protocol::VersionInfo version_info_;
      int address_;
      int debug_request_interval_;

      FlightControlDebugDataCallbackType registered_flight_control_callback_;
      mikrokopter::common::StopWatch timer_flight_control_debug_;
      mikrokopter::common::RunningAverage<double, 3> interval_flight_control_debug_;
      mikrokopter::common::StopWatch stopwatch_debug_request_;

      mikrokopter::protocol::FlightControlDebugData flight_control_debug_data_;
      mikrokopter::protocol::FlightControlDebugDataLabels flight_control_debug_data_labels_;

      void processVersionInfo(const char& command,
                              const int& address,
                              const char* data,
                              const int length);
      void resetVersionInfo();
      bool isValidVersionInfo();

    
      void processFlightControlDebugData(const char& command,
                                         const int& address,
                                         const char* data,
                                         const int length);
      void processFlightControlDebugDataLabels(const char& command,
                                               const int& address,
                                               const char* data,
                                               const int length);
      void printFlightControlDebugData(
                                       const mikrokopter::protocol::FlightControlDebugData& debug_data,
                                       const mikrokopter::protocol::FlightControlDebugDataLabels& debug_labels);


      bool isValidDebugLabel(const std::string& label);
    
      void resetDebugData();

      void initialize();


      void processNaviData(const char& command,
                           const int& address,
                           const char* data,
                           const int length,
                           mikrokopter::protocol::NaviData& navi_data);
    
    
  };
}
                

#endif



