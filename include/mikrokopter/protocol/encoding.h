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
#ifndef KOPTER_PROTOCOL_ENCODING_H_
#define KOPTER_PROTOCOL_ENCODING_H_

#include <string>

#define b64_decode_len(A) (A / 4 * 3 + 2)
#define b64_encode_len(A) ((A+2)/3 * 4 + 1)

namespace mikrokopter
{
  namespace protocol
  {

    /**
     * Base64 encoding of a given byte string
     * @param[out] dest Destination character array to write
     * @param[in] source Character array to be encoded
     * @param[in] length of input character array
     * @return number of bytes in encoded byte string (\a dest)
     */
    int encode64(char* dest, const char* source, size_t len);

    /**
     * Base64 decoding of a given encoded byte string
     * @param[out] dest Destination character array to write
     * @param[in] source Character array to be decoded
     * @param[in] length of encoded character array
     * @return number of bytes in decoded byte string (\a dest)
     */
    int decode64(char* dest, const char* source, size_t len);

    /** Base 64 encode a given character string into a string */
    std::string encode64(const char* s, size_t length);

    /** Base 64 encode a given string into a string */
    std::string encode64(const std::string & s);
  }
}

#endif
