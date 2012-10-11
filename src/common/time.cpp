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
