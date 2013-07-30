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

#include <mikrokopter/io/console.h>

int mikrokopter::io::Console::write(const std::string& message)
{
  PRINT_MESSAGE(message);
  return (int)message.size();
}

int mikrokopter::io::Console::write(const char* message, int length)
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

int mikrokopter::io::Console::read(char* buffer, int max_length)
{
  std::string line;
  std::getline(std::cin, line);
  int nr_bytes_read = std::min((int)line.size(), max_length);
  memcpy(buffer, line.data(), nr_bytes_read);
  return nr_bytes_read;
}

std::string mikrokopter::io::Console::read()
{
  std::string line;
  std::getline(std::cin, line);
  return line;
}
