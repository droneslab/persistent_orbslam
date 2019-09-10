#pragma once

#include <time.h>

namespace performancetools {

    class Timer {
        private:
            struct timespec mBegin;
        public:
            void begin();
            double elapsed();
    };

};
