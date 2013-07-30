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

#include <mikrokopter/common/time.h>

mikrokopter::common::StopWatch::StopWatch()
  : start_time_(boost::posix_time::microsec_clock::local_time())
{
}

double mikrokopter::common::StopWatch::getTime()
{
  boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
  return(static_cast<double>(((end_time - start_time_).total_milliseconds())));
}

double mikrokopter::common::StopWatch::getTimeSeconds()
{
  return(getTime() * 0.001f);
}

void mikrokopter::common::StopWatch::reset()
{
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void mikrokopter::common::StopWatch::set(const double& msec)
{
  start_time_ = boost::posix_time::microsec_clock::local_time() - boost::posix_time::millisec((long)(msec));
}


mikrokopter::common::ScopeTime::ScopeTime(const char* title)
  : title_(std::string(title))
{
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

mikrokopter::common::ScopeTime::~ScopeTime()
{
  double val = this->getTime();
  std::cerr << title_ << " took " << val << "ms.\n";
}


double mikrokopter::common::getTime()
{
  boost::posix_time::ptime epoch_time(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
  return(static_cast<double>((current_time - epoch_time).total_nanoseconds()) * 1.0e-9);
}
