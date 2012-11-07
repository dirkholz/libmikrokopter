#include <mikrokopter/io/serial.h>

mikrokopter::io::Serial::Serial(
    const std::string& port,
    const int& baudrate)
    : port_rx_(), port_tx_(), port_rx_name_(), port_tx_name_(), uart_service_(), stream_buffer_()
    , rx_timeout_(uart_service_)
    , rx_timeout_deadline_(2500) // in ms
    , message_termination_character_("\r\0")
    , connected_(false)
{
  this->connect(port, port, baudrate);
}

mikrokopter::io::Serial::~Serial()
{
  this->close();
}

bool mikrokopter::io::Serial::connect(
    const std::string & port_rx,
    const std::string & port_tx,
    uint32_t baudrate)
{
  if (connected_)
  {
    if ((port_tx.compare(port_tx_name_) == 0) && (port_rx.compare(port_rx_name_)))
      return true;
    this->close();
  }

  connected_ = connect(port_rx_, port_rx, baudrate);
  if (port_rx == port_tx)
    port_tx_ = port_rx_;
  else
    connected_ = connect(port_rx_, port_rx, baudrate);
          
  if (connected_)
  {
    port_tx_name_ = port_tx;
    port_rx_name_ = port_rx;
          
    startReadingRX();
    uart_thread_[0] = boost::thread(boost::bind(&boost::asio::io_service::run,
                                                &uart_service_));
    uart_thread_[1] = boost::thread(boost::bind(&boost::asio::io_service::run,
                                                &uart_service_));
  }
  return connected_;
}

void mikrokopter::io::Serial::close()
{
  uart_service_.post(boost::bind(&boost::asio::deadline_timer::cancel,
                                 &rx_timeout_));
  uart_service_.post(boost::bind(&boost::asio::serial_port::close,
                                 port_rx_));
  if (port_rx_name_ != port_tx_name_)
    uart_service_.post(boost::bind(&boost::asio::serial_port::close,
                                   port_tx_));
  uart_thread_[0].join();
  uart_thread_[1].join();
}

int mikrokopter::io::Serial::write(const std::string& s)
{
  if (DEBUG_LEVEL != 0)
    PRINT_MESSAGE(s);
  return boost::asio::write(*port_tx_,
                            boost::asio::buffer(s.c_str(),
                                                s.size()));
}

int mikrokopter::io::Serial::write(const char* message, int length)
{
  return boost::asio::write(*port_tx_,
                            boost::asio::buffer(message, length));
}


int mikrokopter::io::Serial::read(char* buffer, int max_length)
{
  if (!connected_)
    return 0;
        
  boost::asio::read_until(*port_tx_,
                          stream_buffer_,
                          message_termination_character_);
  std::istream is(&stream_buffer_);
  std::string s;
  // is >> s;
  std::getline(is, s); // hack, for a real solution we need to read until deliminiter

  if (DEBUG_LEVEL != 0)
    PRINT_MESSAGE(s);
        
  int bytes_read = std::min(max_length, (int)s.size());
  memcpy(buffer, s.c_str(), bytes_read);
  return bytes_read;
}
      

std::string mikrokopter::io::Serial::read()
{
  std::string message;
  if (port_rx_->is_open())
  {
    boost::asio::read_until(*port_tx_,
                            stream_buffer_,
                            message_termination_character_);
    std::istream is(&stream_buffer_);
    is >> message;

    if (DEBUG_LEVEL != 0)
      PRINT_MESSAGE(message);
  }
  return message;
}

void mikrokopter::io::Serial::startReadingRX()
{
  if (!connected_)
    return;
        
  boost::asio::async_read_until(*port_tx_,
                                stream_buffer_,
                                message_termination_character_, 
                                boost::bind(&Serial::callbackMessageRX, this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
  if (rx_timeout_deadline_ != 0)
  {
    rx_timeout_.expires_from_now(boost::posix_time::milliseconds(rx_timeout_deadline_));
    rx_timeout_.async_wait(boost::bind(&Serial::callbackTimeoutRX, this, boost::asio::placeholders::error));
  }
}

void mikrokopter::io::Serial::callbackMessageRX(
    const boost::system::error_code& error,
    size_t bytes_transferred)
{
  // process received message
  std::istream is(&stream_buffer_);
  std::string s;
  is >> s;

  if (DEBUG_LEVEL != 0)
    PRINT_MESSAGE(s);

  // reset deadline timeout and restart reading RX
  if (error != boost::asio::error::operation_aborted)
  {
    rx_timeout_.cancel();
    startReadingRX();
  }

  if (registerd_callback_)
    registerd_callback_(s);
}
      
void mikrokopter::io::Serial::callbackTimeoutRX(
    const boost::system::error_code & error)
{
  if (!error)
  {
    port_rx_->cancel();
    std::cerr << __PRETTY_FUNCTION__ << " !!! TIMEOUT !!!"  << std::endl;
  }
}

bool mikrokopter::io::Serial::connect(
    SerialPortPtr & serial_port,
    const std::string & port,
    uint32_t baudrate)
{
  serial_port.reset(new SerialPort(uart_service_));
  serial_port->open(port);

  serial_port->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_port->set_option(boost::asio::serial_port_base::character_size(8));

  return serial_port->is_open();
}
