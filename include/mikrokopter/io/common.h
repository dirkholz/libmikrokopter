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
#ifndef KOPTER_IO_COMMON_H_
#define KOPTER_IO_COMMON_H_

#include <cstdio>
#include <iostream>

#define PRINT_VARIABLE(X) std::cout << #X << " : " << X << std::endl;

#define PRINT_MESSAGE(A) if (A.size() > 0)                              \
  {                                                                     \
    std::cout                                                           \
        << __PRETTY_FUNCTION__ << ": "                                  \
        << #A                                                           \
        << " = "                                                        \
        << mikrokopter::io::removeCarriageReturns(A)                    \
        << " (";                                                        \
        for (size_t i = 0; i < A.size() + 2; ++i)                       \
        {                                                               \
          printf("%x", A.data()[i]);                                    \
          if (i < A.size())                                             \
            printf(" ");                                                \
        }                                                               \
        printf("), %zu bytes\n", A.size());                             \
  }

namespace mikrokopter
{
  namespace io
  {
    /**
     * Remove carriage returns from a string
     * @param[in] String Input string
     * @return Input string with carriage returns removed
     */
    std::string removeCarriageReturns(const std::string& s);
    
  }
}


#endif
