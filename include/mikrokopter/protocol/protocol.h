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

    mikrokopter::protocol::VersionInfo getVersionInfo(const char* data, int length);
  }

}


#endif /* MOD_KOPTER_KOPTER_KOMMUNICATION_H_ */
