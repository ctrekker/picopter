#pragma once

#include <vector>

typedef void (*event_callback)();

class Event {
    private:
        event_callback callback;
        int currentTick;
    public:
        Event(event_callback callback, int period);

        void tick();
};

class EventLoop {
    private:
        int loopTime; // measured in microseconds
        std::vector<Event> events;
    public:
        EventLoop();

        int register_event(Event e);
        void unregister_event(int id);
};
