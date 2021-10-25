#include <chrono>
#include "Timer.h"

Timer::Timer() {

}

void Timer::start() {
    start_time = std::chrono::high_resolution_clock::now();
}
float Timer::stop() {
    std::chrono::duration<float, std::milli> dur = std::chrono::high_resolution_clock::now() - start_time;
    return dur.count();
}
