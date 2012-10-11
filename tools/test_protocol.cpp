#include <mikrokopter/protocol/protocol.h>
#include <mikrokopter/io/console.h>
#include <mikrokopter/io/serial.h>

int main(int argc, char** argv)
{



  // mikrokopter::io::Console::Ptr comm(new mikrokopter::io::Console);

  std::string port = "/dev/ttyUSB2";

  mikrokopter::io::Serial::Ptr comm(new mikrokopter::io::Serial);
  bool connected = comm->connect(port, port, 57600);
  
  comm->write(mikrokopter::protocol::messageSelectNaviCtrl());
  comm->write(mikrokopter::protocol::createMessage('v', 0));
  comm->write(mikrokopter::protocol::messageSelectFlightCtrl());
  comm->write(mikrokopter::protocol::createMessage('v', 0));
  comm->write(mikrokopter::protocol::messageSelectMK3MAG());
  comm->write(mikrokopter::protocol::createMessage('v', 0));

  comm->write(mikrokopter::protocol::messageSelectNaviCtrl());

  int i = 0;
  while (++i < 10)
  {
    comm->write(mikrokopter::protocol::createMessage('v', 0)); // wieso muss die address hier 0 sein?
    sleep(1);
  }


  // mikrokopter::protocol::ControlData ctrl_data;
  // comm->write(mikrokopter::protocol::selectFlightCtrl());
  // comm->write(mikrokopter::protocol::createMessage('b', 1 /* ? */, (char*)&ctrl_data, sizeof(ctrl_data)));
  

  comm->close();
  return 0;
}
  
