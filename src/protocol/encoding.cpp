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

#include <mikrokopter/protocol/encoding.h>

int mikrokopter::protocol::encode64(char* dest, const char* source, size_t len)
{
  if (source == NULL)
    return 0;

  int ptr = 0, pt = 0;
  while(len > 0)
  {
    unsigned char a = 0, b = 0, c = 0;
    if(len) { a = source[ptr++]; len--;}
    if(len) { b = source[ptr++]; len--;}
    if(len) { c = source[ptr++]; len--;}

    dest[pt++] = '=' + (a >> 2);
    dest[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
    dest[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
    dest[pt++] = '=' + ( c & 0x3f);
  }

  return ptr;
}

int mikrokopter::protocol::decode64(char* dest, const char* source, size_t len)
{
  if (source == NULL)
    return 0;

  int offset = 0, ptr = 0;
  while(len != 0)
  {
    const unsigned char a = source[offset++] - '=';
    const unsigned char b = source[offset++] - '=';
    const unsigned char c = source[offset++] - '=';
    const unsigned char d = source[offset++] - '=';

    const unsigned char x = (a << 2) | (b >> 4);
    const unsigned char y = ((b & 0x0f) << 4) | (c >> 2);
    const unsigned char z = ((c & 0x03) << 6) | d;

    if(len--) dest[ptr++] = x; else break;
    if(len--) dest[ptr++] = y; else break;
    if(len--) dest[ptr++] = z; else break;
  }

  return ptr;
}


std::string mikrokopter::protocol::encode64(const char* s, size_t length)
{
  std::string x(b64_encode_len(length), '\0');
  encode64(const_cast<char*>(x.data()), s, length);
  return x;
}

std::string mikrokopter::protocol::encode64(const std::string & s)
{
  return encode64(s.data(), s.size());
}



