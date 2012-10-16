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
#define MEASURE_FUNCTION_TIME                   \
  mikrokopter::common::ScopeTime scopeTime(__func__)
#endif

    /// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code)    \
    if (1) {                                    \
      static double s_lastDone_ = 0.0;          \
      double s_now_ = (currentTime);            \
      if (s_lastDone_ > s_now_)                 \
        s_lastDone_ = s_now_;                   \
      if ((s_now_ - s_lastDone_) > (secs)) {    \
        code;                                   \
        s_lastDone_ = s_now_;                   \
      }                                         \
    } else                                      \
      (void)0
#endif

    /// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code)                            \
    DO_EVERY_TS(secs, mikrokopter::common::getTime(), code)
#endif


#endif
