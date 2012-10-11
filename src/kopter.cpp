#include <mikrokopter/kopter.h>

mikrokopter::Kopter::Kopter(mikrokopter::io::IO::Ptr& comm)
    : comm_(comm)
{
  comm->registerCallback(boost::bind(&mikrokopter::Kopter::parseMessage, this, _1));

  initialize();
}


bool mikrokopter::Kopter::connectNaviCtrl()
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


bool mikrokopter::Kopter::connectFlightCtrl()
{
  comm_->write(mikrokopter::protocol::messageSelectFlightCtrl());
  getVersionInfoBlocking();
  return (address_ == mikrokopter::protocol::ADDRESS_FLIGHT_CTRL);
}


bool mikrokopter::Kopter::connectMK3MAG()
{
  comm_->write(mikrokopter::protocol::messageSelectMK3MAG());
  getVersionInfoBlocking();
  return (address_ == mikrokopter::protocol::ADDRESS_MK3MAG);
}


void mikrokopter::Kopter::getVersionInfoBlocking()
{
  resetVersionInfo();
  while (!isValidVersionInfo())
  {
    comm_->write(mikrokopter::protocol::messageRequestVersion());
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }
}

    
void mikrokopter::Kopter::parseMessage(const std::string& message)
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


void mikrokopter::Kopter::sendExternalControl(const mikrokopter::protocol::ExternControl& control)
{
  comm_->write(mikrokopter::protocol::messageExternalControl(control));
}



void mikrokopter::Kopter::requestDebugData(int interval)
{
  debug_request_interval_ = std::max(interval, 10);
  comm_->write(mikrokopter::protocol::messageRequestFlightControlDebug(debug_request_interval_));
}


void mikrokopter::Kopter::requestDebugDataLabel(const uint8_t& id)
{
  comm_->write(mikrokopter::protocol::messageRequestFlightControlDebugLabels(id));
}


void mikrokopter::Kopter::requestDebugDataLabels()
{
  for (uint8_t i = 0; i < 32; ++i)
    requestDebugDataLabel(i);
}


void mikrokopter::Kopter::registerFlightControlCallback(FlightControlDebugDataCallbackType cb)
{
  registered_flight_control_callback_ = cb;
}



void mikrokopter::Kopter::processVersionInfo(const char& command,
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


void mikrokopter::Kopter::resetVersionInfo()
{
  version_info_.SWMajor = version_info_.SWMinor = version_info_.SWPatch = 0;
}


bool mikrokopter::Kopter::isValidVersionInfo()
{
  return (version_info_.SWMinor != 0);
}


void mikrokopter::Kopter::processFlightControlDebugData(const char& command,
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


void mikrokopter::Kopter::processFlightControlDebugDataLabels(
    const char& command,
    const int& address,
    const char* data,
    const int length)
{
  char label_id = data[0];
  memcpy(const_cast<char*>(flight_control_debug_data_labels_[label_id].data()),
         &data[1],
         mikrokopter::protocol::DEBUG_LABEL_LENGTH);
}


void mikrokopter::Kopter::printFlightControlDebugData(
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


bool mikrokopter::Kopter::isValidDebugLabel(const std::string& label)
{
  return (label.data()[0] != '\0');
}


void mikrokopter::Kopter::resetDebugData()
{
  for (int i = 0; i < 32; ++i)
  {
    flight_control_debug_data_.data[i] = 0;
    flight_control_debug_data_labels_[i] = std::string(mikrokopter::protocol::DEBUG_LABEL_LENGTH, '\0');
  }
}

    
void mikrokopter::Kopter::initialize()
{
  resetVersionInfo();

  resetDebugData();
}
