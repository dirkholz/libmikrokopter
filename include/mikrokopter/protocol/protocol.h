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
#ifndef KOPTER_PROTOCOL_H_
#define KOPTER_PROTOCOL_H_

#include <mikrokopter/protocol/data_types.h>

namespace mikrokopter
{
  namespace protocol
  {

    int calcCRC(const char * message, int length);

    bool checkCRC(const char * message, int length);

    /**
     * @brief create/serialize a message for Mikrokopter communication
     * @param[in] command Single-character command (e.g., 'v' for version info)
     * @param[in] address ?
     * @param[in] data Additional to be sent to the device
     * @param[in] length Size (in characters) of \a data */
    std::string createMessage(const char command,
                              const int address,
                              const char* data,
                              const unsigned int length);

    
    inline std::string createMessage(const char command,
                                     const int address)
    {
      return createMessage(command, address, NULL, 0);
    }

    template <typename T>
    inline std::string createMessage(const char command,
                                     const int address,
                                     const T& data)
    {
      return createMessage(command, address, reinterpret_cast<const char*>(&data), sizeof(data));
    }


    bool parseMessage(const std::string& message,
                      char& command,
                      int& address,
                      char* data,
                      int& length);

    std::string messageSelectNaviCtrl();
    std::string messageSelectFlightCtrl();
    std::string messageSelectMK3MAG();

    std::string messageRequestVersion();

    std::string messageExternalControl(const mikrokopter::protocol::ExternControl& control);

    std::string messageCheckConnection(const uint16_t& value);

    std::string messageRequstErrorMessage();

    std::string messageRequestFlightControlDebug(int interval);
    std::string messageRequestFlightControlDebugLabels(const uint8_t label_id);

    std::string messageRequestNaviControlDebug(int interval);
    
    mikrokopter::protocol::VersionInfo getVersionInfo(const char* data, int length);
  }

}


#endif /* MOD_KOPTER_KOPTER_KOMMUNICATION_H_ */
