#include <mikrokopter/protocol/encoding.h>

int mikrokopter::protocol::encode64(char* dest, const char* source, size_t len)
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

int mikrokopter::protocol::decode64(char* dest, const char* source, size_t len)
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


std::string mikrokopter::protocol::encode64(const char* s, size_t length)
{
  std::string x(b64_encode_len(length), '\0');
  encode64(const_cast<char*>(x.data()), s, length);
  return x;
}

std::string mikrokopter::protocol::encode64(const std::string & s)
{
  return encode64(s.data(), s.size());
}



