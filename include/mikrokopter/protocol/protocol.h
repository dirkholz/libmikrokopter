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
#ifndef KOPTER_PROTOCOL_H_
#define KOPTER_PROTOCOL_H_

#include <cstdio>
#include <cstring>
#include <iostream>
#include <boost/format.hpp>

#include <mikrokopter/io/common.h>
#include <mikrokopter/protocol/data_types.h>
#include <mikrokopter/protocol/encoding.h>

namespace mikrokopter
{
  namespace protocol
  {

    int calcCRC(const char * message, int length)
    {
      int checksum = 0;
      for(int i = 0; i < length; i++)
        checksum += message[i];
      checksum %= 4096;
      return checksum;
    }

    bool checkCRC(const char * message, int length)
    {
      int checksum = calcCRC(message, length-2);
      return ((message[length-2] == ('=' + checksum / 64)) && (message[length-1] == ('=' + checksum % 64)));
    }

    /**
     * @brief create/serialize a message for Mikrokopter communication
     * @param[in] command Single-character command (e.g., 'v' for version info)
     * @param[in] address ?
     * @param[in] data Additional to be sent to the device
     * @param[in] length Size (in characters) of \a data */
    std::string createMessage(const char command,
                              const int address,
                              const char* data,
                              const unsigned int length)
    {
      std::string m = str( boost::format("%1%%2%%3%%4%\0")
                           % '#'
                           % char('a' + address)
                           % command
                           % mikrokopter::protocol::encode64(data, length) );
      int crc = mikrokopter::protocol::calcCRC(m.data(), m.size());
      std::string message = str( boost::format("%1%%2%%3%\r\0")
                                 % m.data()
                                 % char('=' + crc / 64)
                                 % char('=' + crc % 64) );
      return message;
    }


    std::string createMessage(const char command,
                              const int address)
    {
      return createMessage(command, address, NULL, 0);
    }

    template <typename T>
    std::string createMessage(const char command,
                              const int address,
                              const T& data)
    {
      return createMessage(command, address, reinterpret_cast<const char*>(&data), sizeof(data));
    }


    bool parseMessage(const std::string& message,
                      char& command,
                      int& address,
                      char* data,
                      int& length)
    {
      bool checksum_ok = checkCRC(message.data(), message.size());
      if (checksum_ok)
      {
        const char* m = message.data();
        address = char(m[1] - 'a');
        command = m[2];
        int nr_bytes_data = (int)message.size() - 5;
        length = b64_decode_len(nr_bytes_data);
        char decoded_data[length];
        decode64(data, &m[3], nr_bytes_data);
      }
      return checksum_ok;
    }





    inline std::string messageSelectNaviCtrl()
    {
      return std::string(mikrokopter::protocol::MSG_SELECT_NAVI_CTRL);
    }

    inline std::string messageSelectFlightCtrl()
    {
      return createMessage('u', ADDRESS_NAVI_CTRL, new char(0), 1);
    }

    inline std::string messageSelectMK3MAG()
    {
      return createMessage('u', ADDRESS_NAVI_CTRL, new char(1), 1);
    }


    inline std::string messageRequestVersion()
    {
      return createMessage('v', mikrokopter::protocol::ADDRESS_BROADCAST);
    }


    inline std::string messageExternalControl(const mikrokopter::protocol::ExternControl& control)
    {
      // return createMessage('b', mikrokopter::protocol::ADDRESS_FLIGHT_CTRL, control);
      return createMessage('b', mikrokopter::protocol::ADDRESS_BROADCAST, control);
      // NOTE: should have any other address thatn FLIGHT_CTRL!!!
    }




    inline std::string messageCheckConnection(const uint16_t& value)
    {
      return createMessage('z', mikrokopter::protocol::ADDRESS_NAVI_CTRL, value);
    }

    inline std::string messageRequstErrorMessage()
    {
      return createMessage('e', mikrokopter::protocol::ADDRESS_NAVI_CTRL);
    }



    inline std::string messageRequestFlightControlDebug(int interval)
    {
      uint8_t value = ((interval / 10) & 0xFF);
      return createMessage('d', mikrokopter::protocol::ADDRESS_BROADCAST, value);
    }


    inline std::string messageRequestFlightControlDebugLabels(const uint8_t label_id)
    {
      return createMessage('a', mikrokopter::protocol::ADDRESS_BROADCAST, label_id);
    }


    VersionInfo getVersionInfo(const char* data, int length)
    {
      VersionInfo info;
      // info.SWMajor = data[0];
      // info.SWMinor = data[1];
      // info.ProtoMajor = data[2];
      // info.ProtoMinor = data[3];
      // info.SWPatch = data[4];

      // info.HardwareError[0] = data[5];
      // info.HardwareError[1] = data[6];
      // info.HardwareError[2] = data[7];
      // info.HardwareError[3] = data[8];
      // info.HardwareError[4] = data[9];
      memcpy(&info, data, 10);
      return info;
    }



    // uint16_t echo_request_value_sent_;
    // uint16_t echo_request_value_received_;
    // inline uint16_t getRandomEchoRequestValue()
    // {
    //   int myFile = open("/dev/random", O_RDONLY);
    //   uint16_t rand;
    //   int ret = read(myFile, &rand, sizeof(rand));
    //   return ret;
    // }
    // inline void receivedEchoResonse(const )
    // {
    //   printf("%s\n", __PRETTY_FUNCTION__);
    // }



  }

}


#endif /* MOD_KOPTER_KOPTER_KOMMUNICATION_H_ */
