#include <iostream>
#include <mikrokopter/io/console.h>

int main(int argc, char** argv)
{
  mikrokopter::io::Console io;

  int max_buffer_size = 1024;
  char buffer[max_buffer_size];

  std::cout << "mikrokopter::io::console test" << std::endl;
  while (1)
  {
    int nr_bytes_read = io.read(buffer, max_buffer_size);
    int nr_bytes_written = io.write(buffer, nr_bytes_read);
    if (nr_bytes_read != nr_bytes_written)
      break;
  }

  return 0;
}
