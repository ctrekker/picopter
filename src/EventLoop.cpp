#include <algorithm>
#include <vector>
#include <iostream>
#include <unistd.h>

#include "EventLoop.h"
#include "Timer.h"

Event::Event(event_callback callback, int period) {
    this->callback = callback;
    this->period = period;
}
bool Event::tick() {
    currentTick++;
    if(currentTick >= period) {
        currentTick = 0;
        callback();
        return true;
    }
    return false;
}

EventLoop::EventLoop(int loopTime, int64_t maxCycles) {
    this->loopTime = loopTime;
    this->maxCycles = maxCycles;
    cycles = 0;
}

void EventLoop::registerEvent(Event e) {
    events.push_back(e);
}

void EventLoop::run() {
    Timer t;
    while(true) {
        t.start();
        for(int i=0; i<events.size(); i++) {
            events[i].tick();
        }
        cycles++;
        long long execTime = t.stop();

        long long delayTime = loopTime - execTime;
        if(delayTime < 0) {
            std::cout << "WARNING: loop is falling behind (" << (-delayTime) << ")" << std::endl;
        }

        if(maxCycles != -1 && cycles > maxCycles) {
            break;
        }

        usleep(delayTime);
    }
}
