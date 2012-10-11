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
