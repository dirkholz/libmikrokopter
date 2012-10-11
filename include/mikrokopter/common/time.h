#ifndef KOPTER_COMMON_TIME_H_
#define KOPTER_COMMON_TIME_H_

#include <mikrokopter/common/running_average.h>

namespace mikrokopter {
  namespace common {

    class StopWatch
    {
     public:
      StopWatch () : start_time_ (boost::posix_time::microsec_clock::local_time ()) {}

      inline double getTime ()
      {
        boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
        return (static_cast<double> (((end_time - start_time_).total_milliseconds ())));
      }

      inline double getTimeSeconds ()
      {
        return (getTime () * 0.001f);
      }

      inline void reset ()
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }

      inline void set(const double& msec)
      {
        start_time_ = boost::posix_time::microsec_clock::local_time () - boost::posix_time::millisec((long)(msec));
      }
      
     protected:
      boost::posix_time::ptime start_time_;
    };

    // template <int window_size>
    // class RunningAverageTime : public RunningAverage<double, window_size>
    // {
    //  public:
    //   RunningAverageTime() {}
      
    //   virtual double update(double a = 0)
    //   {
    //     buffer_.push_back(timer_.getTime());
    //     return getAverage();
    //   }
      
    //  protected:
      
    //   StopWatch timer_;
    // };
    



    class ScopeTime : public StopWatch
    {
     public:
      inline ScopeTime (const char* title) : title_ (std::string(title))
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }
      
      inline ScopeTime () : title_ (std::string (""))
      {
        start_time_ = boost::posix_time::microsec_clock::local_time ();
      }
      
      inline ~ScopeTime ()
      {
        double val = this->getTime ();
        std::cerr << title_ << " took " << val << "ms.\n";
      }
      
     private:
      std::string title_;
    };
    

#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME                   \
    ScopeTime scopeTime(__func__)
#endif


    inline double getTime ()
    {
      boost::posix_time::ptime epoch_time (boost::gregorian::date (1970, 1, 1));
      boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time ();
      return (static_cast<double>((current_time - epoch_time).total_nanoseconds ()) * 1.0e-9);
    }

    
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
    
  }
}
#endif
