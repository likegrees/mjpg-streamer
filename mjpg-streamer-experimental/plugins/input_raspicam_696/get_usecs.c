#include "get_usecs.h"

int64_t get_usecs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * (int64_t)1000000 + ts.tv_nsec / (int64_t)1000;
}

