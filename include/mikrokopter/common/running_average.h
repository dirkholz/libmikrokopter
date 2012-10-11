#ifndef KOPTER_COMMON_RUNNING_AVERAGE_H_
#define KOPTER_COMMON_RUNNING_AVERAGE_H_

#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>

namespace mikrokopter
{
  namespace common
  {
    template <typename T, int window_size>
    class RunningAverage
    {
     public:
      RunningAverage() : buffer_(window_size) {}
      
      virtual double update(const T& data)
      {
        buffer_.push_back(data);
        return getAverage();
      }
      
      double getAverage()
      {
        if (buffer_.empty()) return 0;
        T sum = 0;
        BOOST_FOREACH(T data, buffer_)
        {
          sum += (double)data;
        }
        return (sum/(double)buffer_.size());
      }
      
     protected:
      boost::circular_buffer<T> buffer_;
    };


    
  }
}


#endif
