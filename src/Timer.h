#pragma once

#include <chrono>

typedef std::chrono::high_resolution_clock::time_point TimeVar;

class Timer {
    private:
        TimeVar start_time;
    public:
        Timer();

        void start();
        float stop();
};
