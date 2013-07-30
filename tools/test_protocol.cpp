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

#include <mikrokopter/protocol/protocol.h>
#include <mikrokopter/io/console.h>
#include <mikrokopter/io/serial.h>

int main(int argc, char** argv)
{
  // mikrokopter::io::Console::Ptr comm(new mikrokopter::io::Console);

  std::string port = "/dev/ttyUSB2";

  mikrokopter::io::Serial::Ptr comm(new mikrokopter::io::Serial);
  bool connected = comm->connect(port, port, 57600);
  if (!connected)
    exit(1);
  
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
  
