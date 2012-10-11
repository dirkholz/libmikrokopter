#include <mikrokopter/kopter.h>
#include <mikrokopter/io/serial.h>
#include <mikrokopter/common/time.h>

int main(int argc, char** argv)
{
  try
  {
    std::string port = "/dev/ttyUSB2";
    if (argc > 1)
      port = argv[1];
    std::cout << "Opening MikroKopter communication on port " << port << std::endl;
    mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Serial(port));
    // mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Console());
    mikrokopter::Kopter kopter(comm);
    
    kopter.connectNaviCtrl();
    // kopter.connectFlightCtrl();

    // kopter.connectMK3MAG();

    // kopter.connectFlightCtrl();

    /*
      left-handed frame, x front
     */
    mikrokopter::protocol::ExternControl control;
    control.config = 1;
    control.pitch = control.roll = 0;
    control.yaw = 0;
    control.height = 0;
    control.gas = 0;

    mikrokopter::common::StopWatch stopwatch_total;

    while(1)
    {
      kopter.sendExternalControl(control);

      const int debug_request_interval = 100;
      DO_EVERY(1, kopter.requestDebugData(debug_request_interval));
      DO_EVERY(1, kopter.requestNaviData(debug_request_interval));

      // DO_EVERY(0.1, kopter.printFlightControlDebugData());

      boost::this_thread::sleep(boost::posix_time::milliseconds(debug_request_interval));
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
