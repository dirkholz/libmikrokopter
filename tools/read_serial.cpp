/** BOOST ASIO SERIAL EXAMPLE

      boost::asio::serial_port
        boost::asio::serial_port::async_read_some

**/

/* compile with
     g++ -o serial serial.cpp -lboost_system -lboost_thread
*/

#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

class Serial {

  char read_msg_[512];

  boost::asio::io_service m_io;

  boost::asio::serial_port m_port;

 private:

  void handler(  const boost::system::error_code& error, size_t bytes_transferred)
  {
    read_msg_[bytes_transferred]=0;
    std::cout << bytes_transferred << " bytes: " << read_msg_ << std::endl;

    read_some();
  }

  void read_some()
  {
    m_port.async_read_some(boost::asio::buffer(read_msg_,512),
                           boost::bind(&Serial::handler,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred) );
  }

 public:

  Serial(const char *dev_name) : m_io(), m_port(m_io, dev_name)
  {
    /*
      port.set_option( boost::asio::serial_port_base::parity() );// default none
            port.set_option( boost::asio::serial_port_base::character_size( 8 ) );
            port.set_option( boost::asio::serial_port_base::stop_bits() );// default one
                  port.set_option( boost::asio::serial_port_base::baud_rate( baud_rate ) );
    */
    read_some();

    // run the IO service as a separate thread, so the main thread can do others
    boost::thread t(boost::bind(&boost::asio::io_service::run, &m_io));
  }

};

/* serial <devicename> */

int main(int argc, char* argv[])
{
  Serial s(argv[1]);

  // wait some
  sleep(10);

  return 0;

} 
