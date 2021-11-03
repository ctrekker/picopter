#include <chrono>
#include "Timer.h"

Timer::Timer() {

}

void Timer::start() {
    start_time = std::chrono::high_resolution_clock::now();
}
long long Timer::stop() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}
