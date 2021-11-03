#include <iostream>
#include "Integrators.h"

// TODO: Remove main method during pi integration
int main() {
    float dt = 0.1;

    TrapezoidalIntegrator<float> trapInt = TrapezoidalIntegrator<float>(dt);
    Integrator<float>* integrator = dynamic_cast<Integrator<float>*>(&trapInt);

    float maxTime = 1;

    for(int _t=0; _t<(maxTime / dt + 1); _t++) {
        float t = (float)_t * dt;
        integrator->step(t * t);
    }

    std::cout << integrator->result() << std::endl;

    return 0;
}


template <typename T>
Integrator<T>::Integrator(T dt) {
    this->dt = dt;
}

template <typename T>
void TrapezoidalIntegrator<T>::step(T nextValue) {
    if(!hasFirstValue) {
        firstValue = nextValue;
        hasFirstValue = true;
    }
    else {
        aggregate += nextValue * Integrator<T>::dt;
    }
    lastValue = nextValue;
}

template <typename T>
T TrapezoidalIntegrator<T>::result() {
    return aggregate + Integrator<T>::dt * (lastValue + firstValue) / 2;
}
