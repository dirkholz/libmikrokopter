#include <mikrokopter/protocol/protocol.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <boost/format.hpp>

#include <mikrokopter/io/common.h>
#include <mikrokopter/protocol/data_types.h>
#include <mikrokopter/protocol/encoding.h>

int mikrokopter::protocol::calcCRC(const char * message, int length)
{
  int checksum = 0;
  for(int i = 0; i < length; i++)
    checksum += message[i];
  checksum %= 4096;
  return checksum;
}

bool mikrokopter::protocol::checkCRC(const char * message, int length)
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
std::string mikrokopter::protocol::createMessage(const char command,
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


bool mikrokopter::protocol::parseMessage(const std::string& message,
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





std::string mikrokopter::protocol::messageSelectNaviCtrl()
{
  return std::string(mikrokopter::protocol::MSG_SELECT_NAVI_CTRL);
}

std::string mikrokopter::protocol::messageSelectFlightCtrl()
{
  return createMessage('u', ADDRESS_NAVI_CTRL, new char(0), 1);
}

std::string mikrokopter::protocol::messageSelectMK3MAG()
{
  return createMessage('u', ADDRESS_NAVI_CTRL, new char(1), 1);
}


std::string mikrokopter::protocol::messageRequestVersion()
{
  return createMessage('v', mikrokopter::protocol::ADDRESS_BROADCAST);
}


std::string mikrokopter::protocol::messageExternalControl(const mikrokopter::protocol::ExternControl& control)
{
  return createMessage('b', mikrokopter::protocol::ADDRESS_BROADCAST, control);
}




std::string mikrokopter::protocol::messageCheckConnection(const uint16_t& value)
{
  return createMessage('z', mikrokopter::protocol::ADDRESS_NAVI_CTRL, value);
}

std::string mikrokopter::protocol::messageRequstErrorMessage()
{
  return createMessage('e', mikrokopter::protocol::ADDRESS_NAVI_CTRL);
}



std::string mikrokopter::protocol::messageRequestFlightControlDebug(int interval)
{
  uint8_t value = ((interval / 10) & 0xFF);
  return createMessage('d', mikrokopter::protocol::ADDRESS_BROADCAST, value);
}


std::string mikrokopter::protocol::messageRequestFlightControlDebugLabels(const uint8_t label_id)
{
  return createMessage('a', mikrokopter::protocol::ADDRESS_BROADCAST, label_id);
}


mikrokopter::protocol::VersionInfo mikrokopter::protocol::getVersionInfo(const char* data, int length)
{
  VersionInfo info;
  memcpy(&info, data, 10);
  return info;
}
