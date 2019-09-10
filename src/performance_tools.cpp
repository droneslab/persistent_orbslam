#include "performance_tools.h"

namespace performancetools {

    void Timer::begin() {
        clock_gettime(CLOCK_MONOTONIC, &mBegin);
    }

    double Timer::elapsed() {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        return (now.tv_sec - mBegin.tv_sec) + (now.tv_nsec - mBegin.tv_nsec) * 1e-9;
    }

};
