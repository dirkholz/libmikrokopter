/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Dirk Holz, University of Bonn.
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
      
      Serial(const std::string& port = "/dev/mikrokopter",
             const int& baudrate = 57600);

      virtual ~Serial();

      /**
       * Connect to serial port(s)
       * @param[in] port_tx Serial port to open for reading data
       * @param[in] port_tx Serial port to open for writing data
       * @param[in] Baudrate to use for communication
       * @return true if serial port(s) was successfully opened, false otherwise.
       */
      bool connect(const std::string & port_rx, const std::string & port_tx, uint32_t baudrate);

      /** Close serial port conenction. */
      void close();

      
      virtual int write(const std::string& s);
      virtual int write(const char* message, int length);
      virtual int read(char* buffer, int max_length);
      virtual std::string read();

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

      /** start asynchronous reading */
      void startReadingRX();

      /** callback when data was read during asynchronous reading */
      void callbackMessageRX(const boost::system::error_code& error, size_t bytes_transferred);
      
      /** callback when timeout occured during asynchronous reading */
      void callbackTimeoutRX(const boost::system::error_code & error);

      /**
       * Open serial port connection
       * @param[in] serial_port Internal communication port to use.
       * @param[in] port Serial port to open
       * @param[in] baudrate Baud rate ot use
       */
      bool connect(SerialPortPtr & serial_port, const std::string & port, uint32_t baudrate);
      
    };

  }
}
#endif
