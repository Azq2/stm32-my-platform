#include <cstdint>

typedef int (StdioReadCallback)(int file, char *s, int size);
typedef int (StdioWriteCallback)(int file, const char *s, int size);

void libc_set_write_callback(StdioWriteCallback *callback);
void libc_set_read_callback(StdioReadCallback *callback);
