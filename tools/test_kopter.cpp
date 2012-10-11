#include <mikrokopter/kopter.h>
#include <mikrokopter/io/serial.h>
#include <mikrokopter/io/console.h>

int main(int argc, char** argv)
{
  try
  {
    const std::string port = "/dev/ttyUSB2";
    std::cout << "Opening MikroKopter communication on port " << port << std::endl;
    mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Serial(port));
    // mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Console());
    mikrokopter::Kopter kopter(comm);
    
    // kopter.connectNaviCtrl();
    // kopter.connectFlightCtrl();
    // kopter.connectMK3MAG();

    // connect to FlightCtrl and request debug data
    kopter.connectFlightCtrl();
    const int debug_request_interval = 50;
    kopter.requestDebugData(debug_request_interval);

    
    /*
      left-handed frame, x front
     */
    mikrokopter::protocol::ExternControl control;
    control.config = 0;
    control.pitch = control.roll = 0;
    control.yaw = 0;
    control.height = 0;
    control.gas = 0;
    while(1)
    {
      // kopter.sendExternalControl(control);

      DO_EVERY(1, kopter.requestDebugData(50));
      DO_EVERY(0.1, kopter.printFlightControlDebugData());

      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    
  }
  catch( boost::system::system_error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl;
  }
  catch(...)
  {
    std::cerr << "Caught unhandled exception!" << std::endl;
  }
  
  return 0;
}
