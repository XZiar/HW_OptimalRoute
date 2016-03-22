#pragma once

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <chrono>
#define SSE

#if defined(SSE)
#    if defined(__GNUC__)
#        include <x86intrin.h>
#        define _MM_ALIGN32 __attribute__((aligned(32)))
#        define malloc_align(size, align) memalign((align), (size))
#        define free_align(ptr) free(ptr)
#    else
#        include <intrin.h>
#        define _MM_ALIGN32 __declspec(align(32))
#        define malloc_align(size, align) _aligned_malloc((size), (align))
#        define free_align(ptr) _aligned_free(ptr)
#    endif
#    define AVX
#endif


using namespace std;

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))