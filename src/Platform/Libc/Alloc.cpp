#include <FreeRTOS.h>
#include <new>
#include <cstring>
#include <cerrno>

/*
 * Use FreeRTOS for memory allocation
 * */
extern "C"
__attribute__((used))
void *malloc(size_t size) {
	return pvPortMalloc(size);
}

extern "C"
__attribute__((used))
void free(void *ptr) {
    vPortFree(ptr);
}

extern "C"
__attribute__((used))
void *realloc(void *ptr, size_t n) {
	void *p = pvPortMalloc(n);
	if (ptr)
		memcpy(p, ptr, n);
	vPortFree(ptr);
	return p;
}

extern "C"
__attribute__((used))
void *calloc(size_t __nmemb, size_t __size) {
	void *p = pvPortMalloc(__nmemb * __size);
	memset(p, 0, __nmemb * __size);
	return p;
}

extern "C"
__attribute__((used))
void *_malloc_r(struct _reent *r, size_t sz) {
	(void) r;
	return malloc(sz);
}

extern "C"
__attribute__((used))
void *_calloc_r(struct _reent *r, size_t a, size_t b) {
	(void) r;
	return calloc(a, b);
}

extern "C"
__attribute__((used))
void _free_r(struct _reent *r, void *x) {
	(void) r;
	free(x);
}

extern "C"
__attribute__((used))
void *_realloc_r(struct _reent *r, void *x, size_t sz) {
	(void) r;
	return realloc(x, sz);
}

__attribute__((used))
void *operator new(std::size_t count) {
	return pvPortMalloc(count);
}

__attribute__((used))
void *operator new[](std::size_t count) {
	return pvPortMalloc(count);
}

__attribute__((used))
void operator delete(void *ptr) noexcept {
	vPortFree(ptr);
}

__attribute__((used))
void operator delete(void *ptr, std::size_t) noexcept {
	vPortFree(ptr);
}

__attribute__((used))
void operator delete[](void *ptr) noexcept {
	vPortFree(ptr);
}

__attribute__((used))
void operator delete[](void *ptr, std::size_t) noexcept {
	vPortFree(ptr);
}
