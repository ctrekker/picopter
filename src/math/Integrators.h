#pragma once

template <typename T>
class Integrator {
    protected:
        T dt;
    public:
        Integrator(T dt);

        virtual void step(T nextValue) = 0;
        virtual T result() = 0;
};

template <typename T>
class TrapezoidalIntegrator: public Integrator<T> {
    private:
        bool hasFirstValue = false;
        T firstValue = 0.;
        T lastValue = 0.;
        T aggregate = 0.;
    public:
        TrapezoidalIntegrator(T dt) : Integrator<T>(dt) {};

        void step(T nextValue);
        T result();
};

template <typename T>
class SimpsonIntegrator: public Integrator<T> {
    private:
        uint64_t currentStep = 0;
        bool hasFirstValue = false;
        int sumState = 0;
        T firstValue = 0;
        T aggregate = 0;

        T values[3]{0, 0, 0};
    public:
        SimpsonIntegrator(T dt) : Integrator<T>(dt) {};

        void step(T nextValue);
        T result();
};
