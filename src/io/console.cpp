#include <mikrokopter/io/console.h>

int mikrokopter::io::Console::write(const std::string& message)
{
  PRINT_MESSAGE(message);
  return (int)message.size();
}

int mikrokopter::io::Console::write(const char* message, int length)
{
  printf("%s ", __PRETTY_FUNCTION__);

  // for (int i = 0; i < length; ++i)
  //   std::cout << message[i];

  std::cout << " (";
  for (int i = 0; i < length; ++i)
  {
    printf("%x", message[i]);
    if (i != length-1)
      std::cout << ' ';
  }
  std::cout << ')';

  std::cout << std::endl;
  return length;
}

int mikrokopter::io::Console::read(char* buffer, int max_length)
{
  std::string line;
  std::getline(std::cin, line);
  int nr_bytes_read = std::min((int)line.size(), max_length);
  memcpy(buffer, line.data(), nr_bytes_read);
  return nr_bytes_read;
}

std::string mikrokopter::io::Console::read()
{
  std::string line;
  std::getline(std::cin, line);
  return line;
}
