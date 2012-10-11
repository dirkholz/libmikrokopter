#include <mikrokopter/io/common.h>

std::string mikrokopter::io::removeCarriageReturns(const std::string& s)
{
  std::string result(s.size(), '\0');
  char* dst = const_cast<char*>(result.data());
  char* str = const_cast<char*>(s.data());
  
  for (char* src = str; *src != '\0'; src++) {
    *dst = *src;
    if (*dst != '\r') dst++;
  }
  *dst = '\0';
  return result;
}
