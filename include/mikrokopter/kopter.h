/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dirk Holz, dirk.holz@ieee.org
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *
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
    Kopter(mikrokopter::io::IO::Ptr& comm)
        : comm_(comm)
    {
      comm->registerCallback(boost::bind(&mikrokopter::Kopter::parseMessage, this, _1));

      initialize();
    }
    
    bool connectNaviCtrl()
    {

      char data[6];
      data[0] = 27;
      data[1] = 27;
      data[2] = 85;
      data[3] = -86;
      data[4] = '\r';
      data[5] = '\0';
      comm_->write(data, sizeof(data));

      // comm_->write(mikrokopter::protocol::messageSelectNaviCtrl());
      getVersionInfoBlocking();
      return (address_ == mikrokopter::protocol::ADDRESS_NAVI_CTRL);
    }
    
    bool connectFlightCtrl()
    {
      comm_->write(mikrokopter::protocol::messageSelectFlightCtrl());
      getVersionInfoBlocking();
      return (address_ == mikrokopter::protocol::ADDRESS_FLIGHT_CTRL);
    }
    
    bool connectMK3MAG()
    {
      comm_->write(mikrokopter::protocol::messageSelectMK3MAG());
      getVersionInfoBlocking();
      return (address_ == mikrokopter::protocol::ADDRESS_MK3MAG);
    }

    void getVersionInfoBlocking()
    {
      resetVersionInfo();
      while (!isValidVersionInfo())
      {
        comm_->write(mikrokopter::protocol::messageRequestVersion());
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      }
    }

    
    void parseMessage(const std::string& message)
    {
      if (message.size() == 0)
        return;
      
      // PRINT_MESSAGE(message);

      char command;
      int address;
      char buffer[1024];
      int length;
      bool valid = mikrokopter::protocol::parseMessage(message,
                                                  command,
                                                  address,
                                                  buffer,
                                                  length);
      if (!valid)
      {
        std::cerr << __PRETTY_FUNCTION__ << ": CRC mismatch:" << message << "!\n";
        return;
      }

      const bool print_message = false;
      if (print_message)
      {
        std::cout << __PRETTY_FUNCTION__ << " ";
        if (address == mikrokopter::protocol::ADDRESS_NAVI_CTRL)
          std::cout << "NaviCtrl: ";
        if (address == mikrokopter::protocol::ADDRESS_FLIGHT_CTRL)
          std::cout << "FlightCtrl: ";
        if (address == mikrokopter::protocol::ADDRESS_MK3MAG)
          std::cout << "MK3MAG: ";
        std::cout << "\' " << command << "\' "
                  << " with " << length
                  << " data bytes ( ";
        for (int i = 0; i < length; ++i)
          printf("%x ", buffer[i]);
        printf(")\n");
      }
      
      address_ = address;
      switch (command)
      {
     case 'V': // VERSION reply
       processVersionInfo(command, address, buffer, length);
       break;
     case 'D': // Debug data (continuously sent (requested interval)
       interval_flight_control_debug_.update(timer_flight_control_debug_.getTime());
       timer_flight_control_debug_.reset();
       processFlightControlDebugData(command, address, buffer, length);
       break;
     case 'A': // label for debug data
       processFlightControlDebugDataLabels(command, address, buffer, length);
       break;
       
      }
    }
    
    mikrokopter::io::IO::Ptr getComm() { return comm_; }


    void sendExternalControl(const mikrokopter::protocol::ExternControl& control)
    {
      comm_->write(mikrokopter::protocol::messageExternalControl(control));
    }


    void requestDebugData(int interval)
    {
      debug_request_interval_ = std::max(interval, 10);
      comm_->write(mikrokopter::protocol::messageRequestFlightControlDebug(debug_request_interval_));
    }

    inline void requestDebugDataLabel(const uint8_t& id)
    {
      comm_->write(mikrokopter::protocol::messageRequestFlightControlDebugLabels(id));
    }
    
    void requestDebugDataLabels()
    {
      for (uint8_t i = 0; i < 32; ++i)
        requestDebugDataLabel(i);
    }

    inline void printFlightControlDebugData()
    {
      printFlightControlDebugData(flight_control_debug_data_,
                                  flight_control_debug_data_labels_);
    }

    inline mikrokopter::protocol::FlightControlDebugData getDebugData() { return flight_control_debug_data_; }
    
    void registerFlightControlCallback(FlightControlDebugDataCallbackType cb)
    {
      registered_flight_control_callback_ = cb;
    }
    
   protected:

    mikrokopter::io::IO::Ptr comm_;


    

    mikrokopter::protocol::VersionInfo version_info_;
    int address_;
    void processVersionInfo(const char& command,
                            const int& address,
                            const char* data,
                            const int length)
    {
      version_info_ = mikrokopter::protocol::getVersionInfo(data, length);
      
      if (address == mikrokopter::protocol::ADDRESS_NAVI_CTRL)
        printf("Connected to NaviCtrl, ");
      else if (address == mikrokopter::protocol::ADDRESS_FLIGHT_CTRL)
        printf("Connected to FlightCtrl, ");
      else if (address == mikrokopter::protocol::ADDRESS_MK3MAG)
        printf("Connected to MK3MAG, ");
      else return; // must be a weird case!
      printf("software version: %d.%d%c, protocol version: %d.%d\n",
             version_info_.SWMajor, version_info_.SWMinor, ('a' + version_info_.SWPatch),
             version_info_.ProtoMajor, version_info_.ProtoMinor);
    }
    inline void resetVersionInfo()
    {
      version_info_.SWMajor = version_info_.SWMinor = version_info_.SWPatch = 0;
    }
    inline bool isValidVersionInfo()
    {
      return (version_info_.SWMinor != 0);
    }



    FlightControlDebugDataCallbackType registered_flight_control_callback_;
    mikrokopter::common::StopWatch timer_flight_control_debug_;
    mikrokopter::common::RunningAverage<double, 3> interval_flight_control_debug_;
    mikrokopter::protocol::FlightControlDebugData flight_control_debug_data_;
    int debug_request_interval_;
    mikrokopter::common::StopWatch stopwatch_debug_request_;
    void processFlightControlDebugData(const char& command,
                                       const int& address,
                                       const char* data,
                                       const int length)
    {
      memcpy(&flight_control_debug_data_, data, sizeof(flight_control_debug_data_));
      // std::cout << __PRETTY_FUNCTION__ << std::endl << flight_control_debug_data_ << std::endl;

      // re-request debug data (to avoid that the flight control stops sending)
      if (stopwatch_debug_request_.getTime() > 1000.0)
        comm_->write(mikrokopter::protocol::messageRequestFlightControlDebug(debug_request_interval_));


      if (registered_flight_control_callback_)
        registered_flight_control_callback_(flight_control_debug_data_);
    }

    mikrokopter::protocol::FlightControlDebugDataLabels flight_control_debug_data_labels_;
    void processFlightControlDebugDataLabels(const char& command,
                                             const int& address,
                                             const char* data,
                                             const int length)
    {
      char label_id = data[0];
      memcpy(const_cast<char*>(flight_control_debug_data_labels_[label_id].data()),
             &data[1],
             mikrokopter::protocol::DEBUG_LABEL_LENGTH);
    }
        
    inline void printFlightControlDebugData(
        const mikrokopter::protocol::FlightControlDebugData& debug_data,
        const mikrokopter::protocol::FlightControlDebugDataLabels& debug_labels)
    {
      std::cout << std::endl << __PRETTY_FUNCTION__ << std::endl;
      std::cout << "Update Interval: " << interval_flight_control_debug_.getAverage() << "ms." << std::endl;
      for (uint8_t i = 0; i < 32; ++i)
      {
        if (!isValidDebugLabel(debug_labels[i]))
          requestDebugDataLabel(i);
        std::cout << debug_labels[i] 
                  << " = "
                  << debug_data.data[i] << std::endl;
      }
    }

    inline bool isValidDebugLabel(const std::string& label)
    {
      return (label.data()[0] != '\0');
    }
    
    inline void resetDebugData()
    {
      for (int i = 0; i < 32; ++i)
      {
        flight_control_debug_data_.data[i] = 0;
        flight_control_debug_data_labels_[i] = std::string(mikrokopter::protocol::DEBUG_LABEL_LENGTH, '\0');
      }
    }

    
    inline void initialize()
    {
      resetVersionInfo();

      resetDebugData();
    }

  };
}
                

#endif



