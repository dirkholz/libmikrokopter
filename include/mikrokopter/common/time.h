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
#ifndef KOPTER_COMMON_TIME_H_
#define KOPTER_COMMON_TIME_H_

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace mikrokopter {
  namespace common {

    class StopWatch
    {
      public:
        StopWatch();
        virtual ~StopWatch(){};
      
        double getTime();
        double getTimeSeconds();
        void reset();
        void set(const double& msec);
      
      protected:
        boost::posix_time::ptime start_time_;
    };

    class ScopeTime : public StopWatch
    {
      public:
        ScopeTime(const char* title = "");
        ~ScopeTime();
      
      private:
        std::string title_;
    };
    
    double getTime();
  }
}

    
#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME                         \
  mikrokopter::common::ScopeTime scopeTime(__func__)
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code)    \
  if (1) {                                      \
    static double s_lastDone_ = 0.0;            \
    double s_now_ = (currentTime);              \
    if (s_lastDone_ > s_now_)                   \
      s_lastDone_ = s_now_;                     \
    if ((s_now_ - s_lastDone_) > (secs)) {      \
      code;                                     \
      s_lastDone_ = s_now_;                     \
    }                                           \
  } else                                        \
    (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code)                              \
  DO_EVERY_TS(secs, mikrokopter::common::getTime(), code)
#endif


#endif
