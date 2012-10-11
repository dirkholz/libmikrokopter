/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/* DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *                    Version 2, December 2004
 *
 *  Copyright (c) 2012, Dirk Holz, dirk.holz@ieee.org
 *
 * Everyone is permitted to copy and distribute verbatim or modified
 * copies of this license document, and changing it is allowed as long
 * as the name is changed.
 *
 *            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 *
 *  0. You just DO WHAT THE FUCK YOU WANT TO.
 *
 * This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://sam.zoy.org/wtfpl/COPYING for more details.
 *
 */
#ifndef KOPTER_PROTOCOL_ENCODING_H_
#define KOPTER_PROTOCOL_ENCODING_H_

#define b64_decode_len(A) (A / 4 * 3 + 2)
#define b64_encode_len(A) ((A+2)/3 * 4 + 1)

namespace mikrokopter
{
  namespace protocol
  {

    int encode64(char* dest, const char* source, size_t len)
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

    int decode64(char* dest, const char* source, size_t len)
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


    std::string encode64(const char* s, size_t length)
    {
      std::string x(b64_encode_len(length), '\0');
      encode64(const_cast<char*>(x.data()), s, length);
      return x;
    }

    inline std::string encode64(const std::string & s)
    {
      return encode64(s.data(), s.size());
    }


  }
}

#endif
