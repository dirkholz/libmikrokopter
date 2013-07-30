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

#include <mikrokopter/kopter.h>
#include <mikrokopter/io/console.h>
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
    // mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Serial(port));
    mikrokopter::io::IO::Ptr comm(new mikrokopter::io::Console());
    mikrokopter::Kopter kopter(comm);

    // comm->DEBUG_LEVEL = 1;
    
    // kopter.connectNaviCtrl();
    kopter.connectFlightCtrl();
    // kopter.connectMK3MAG();

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

      const int debug_request_interval = 50;
      DO_EVERY(1, kopter.requestDebugData(debug_request_interval));
      DO_EVERY(1, kopter.requestNaviData(debug_request_interval));

      DO_EVERY(0.1, kopter.printFlightControlDebugData());

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
