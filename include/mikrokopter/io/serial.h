/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dirk Holz, dirk.holz@ieee.org
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *
 */
#ifndef KOPTER_IO_SERIAL_H_
#define KOPTER_IO_SERIAL_H_

#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <mikrokopter/io/io.h>

namespace mikrokopter
{
  namespace io
  {
    
    class Serial : public mikrokopter::io::IO
    {
     public:

      typedef boost::shared_ptr<Serial> Ptr;
      
      // Serial()
      Serial(const std::string& port = "/dev/mikrokopter",
             const int& baudrate = 57600)
          : rx_timeout_(uart_service_)
          , rx_timeout_deadline_(2500) // in ms
          , message_termination_character_("\r\0")
          , connected_(false)
      {
        this->connect(port, port, baudrate);
      }

      ~Serial()
      {
        this->close();
      }

      bool connect(const std::string & port_rx, const std::string & port_tx, uint32_t baudrate)
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

      void close()
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
      
      virtual int write(const std::string& s)
      {
        // PRINT_MESSAGE(s);
        return boost::asio::write(*port_tx_,
                                  boost::asio::buffer(s.c_str(),
                                                      s.size()));
      }
      
      virtual int write(const char* message, int length)
      {
        return boost::asio::write(*port_tx_,
                                  boost::asio::buffer(message, length));
      }

      virtual int read(char* buffer, int max_length)
      {
        if (!connected_)
          return 0;
        
        boost::asio::read_until(*port_tx_,
                                stream_buffer_,
                                message_termination_character_);
        std::istream is(&stream_buffer_);
        std::string s;
        is >> s;
        PRINT_MESSAGE(s);
        
        int bytes_read = std::min(max_length, (int)s.size());
        memcpy(buffer, s.c_str(), bytes_read);
        return bytes_read;
      }
      

      virtual std::string read()
      {
        std::string message;
        if (port_rx_->is_open())
        {
          boost::asio::read_until(*port_tx_,
                                  stream_buffer_,
                                  message_termination_character_);
          std::istream is(&stream_buffer_);
          is >> message;
          PRINT_MESSAGE(message);
        }
        return message;
      }

     private:

      typedef boost::asio::serial_port SerialPort;
      typedef boost::shared_ptr<SerialPort> SerialPortPtr;
      SerialPortPtr port_rx_;
      SerialPortPtr port_tx_;
      boost::asio::io_service uart_service_;
      boost::asio::deadline_timer rx_timeout_;
      int rx_timeout_deadline_;
      std::string message_termination_character_;
      bool connected_;

      boost::thread uart_thread_[2];
      std::string port_rx_name_;
      std::string port_tx_name_;

      boost::asio::streambuf stream_buffer_;

      void startReadingRX()
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

      
      void callbackMessageRX(const boost::system::error_code& error, size_t bytes_transferred)
      {
        // process received message
        std::istream is(&stream_buffer_);
        std::string s;
        is >> s;

        // PRINT_MESSAGE(s);

        // reset deadline timeout and restart reading RX
        if (error != boost::asio::error::operation_aborted)
        {
          rx_timeout_.cancel();
          startReadingRX();
        }

        if (registerd_callback_)
          registerd_callback_(s);
      }
      
      void callbackTimeoutRX(const boost::system::error_code & error)
      {
        if (!error)
        {
          port_rx_->cancel();
          std::cerr << __PRETTY_FUNCTION__ << " !!! TIMEOUT !!!"  << std::endl;
        }
      }

      bool connect(SerialPortPtr & serial_port, const std::string & port, uint32_t baudrate)
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


      
    };

  }
}
#endif
