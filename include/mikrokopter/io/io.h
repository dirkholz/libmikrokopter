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
#ifndef KOPTER_IO_IO_H_
#define KOPTER_IO_IO_H_

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <mikrokopter/io/common.h>

namespace mikrokopter
{
  namespace io
  {
    /** Abstract interface and base class for IO functionality */
    class IO
    {
      public:
        typedef boost::shared_ptr<mikrokopter::io::IO> Ptr;
        typedef boost::shared_ptr<const mikrokopter::io::IO> ConstPtr;
        typedef boost::function<void(const std::string&)> CallbackType;

        /** Empty constructor */
        IO()
          : DEBUG_LEVEL(0)
          , registerd_callback_(boost::bind(&mikrokopter::io::IO::defaultCallback, this, _1))
        {};

        virtual ~IO(){}
      
        /**
         * Output: write message
         * @param[in] message Message to write
         * @param[in] length Message length in characters
         * @return number of bytes written (should be equal to message length!)
         */
        virtual int write(const char* message, int length) = 0;

        /**
         * Output: write message
         * @param[in] message Message to write
         * @return number of bytes written (should be equal to message length!)
         */
        virtual int write(const std::string& message)
        {
          return write(message.data(), message.size());
        }

        /**
         * Input: read message
         * @param[out] buffer to fill with read message
         * @param[in] max_length Size of buffer (maximum number of characters to read)
         * @return number of bytes read
         */
        virtual int read(char* buffer, int max_length) = 0;

        /**
         * Input: Read message as string.
         * @return Read string
         */
        virtual std::string read() = 0;

        /**
         * Register a callback for those IO classes that support asynchronous reading
         * @param[in] cb Function to register
         */
        void registerCallback(CallbackType cb)
        {
          registerd_callback_ = cb;
        }


        int DEBUG_LEVEL; // no debug output

      protected:

        CallbackType registerd_callback_;

        /** Default function used when no callback is registered */
        virtual void defaultCallback(const std::string& s)
        {
          PRINT_MESSAGE(s);
        }

    };
    
  }
}

#endif

