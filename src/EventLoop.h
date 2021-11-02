#pragma once

#include <vector>

typedef void (*event_callback)();

class Event {
    private:
        event_callback callback;
        int period;
        int currentTick = 0;
    public:
        Event(event_callback callback, int period);

        bool tick();
};

class EventLoop {
    private:
        int loopTime; // measured in microseconds
        std::vector<Event> events;
        int64_t cycles;
        int64_t maxCycles;
    public:
        EventLoop(int loopTime, int64_t maxCycles=-1);

        void registerEvent(Event e);
        void run();
};
