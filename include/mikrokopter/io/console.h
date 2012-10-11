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
#ifndef KOPTER_IO_CONSOLE_H_
#define KOPTER_IO_CONSOLE_H_

#include <cstdio>
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <mikrokopter/io/io.h>

namespace mikrokopter
{
  namespace io
  {

    /** Console IO for testing/debug purposes */
    class Console : public mikrokopter::io::IO
    {
      
     public:
      Console() {};

      // inheritance of damn overladed inline function wouldn't work otherwise
      int write(const std::string& message)
      {
        // std::cout << message << std::endl;
        PRINT_MESSAGE(message);
      }
      
      int write(const char* message, int length)
      {
        printf("%s ", __PRETTY_FUNCTION__);

        // for (int i = 0; i < length; ++i)
        //   std::cout << message[i];

        std::cout << " (";
        for (int i = 0; i < length; ++i)
        {
          printf("%x", message[i]);
          if (i != length-1)
            std::cout << ' ';
        }
        std::cout << ')';

        std::cout << std::endl;
        return length;
      }

      int read(char* buffer, int max_length)
      {
        std::string line;
        std::getline(std::cin, line);
        int nr_bytes_read = std::min((int)line.size(), max_length);
        memcpy(buffer, line.data(), nr_bytes_read);
        return nr_bytes_read;
      }

      virtual std::string read()
      {
        std::string line;
        std::getline(std::cin, line);
        return line;
      }

      
    };
    
  }
}

#endif

