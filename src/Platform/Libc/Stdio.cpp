#include "Stdio.h"

#include <cstdio>
#include <cerrno>

static StdioWriteCallback *stdio_write_callback = nullptr;
static StdioReadCallback *stdio_read_callback = nullptr;

void libc_set_write_callback(StdioWriteCallback *callback) {
	stdio_write_callback = callback;
}

void libc_set_read_callback(StdioReadCallback *callback) {
	stdio_read_callback = callback;
}

/*
 * Hooks for standart libc IO
 * */
extern "C"
__attribute__((used))
int _write(int file, char *ptr, int len) {
	if (stdio_write_callback)
		return stdio_write_callback(file, ptr, len);
	errno = EIO;
	return -1;
}

extern "C"
__attribute__((used))
int _read(int file, char *ptr, int len) {
	if (stdio_read_callback)
		return stdio_read_callback(file, ptr, len);
	errno = EIO;
	return -1;
}
