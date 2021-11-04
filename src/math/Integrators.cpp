#include <iostream>
#include "Integrators.h"
#include <math.h>

// TODO: Remove main method during pi integration
int main() {
    float dt = 0.1;

    SimpsonIntegrator<float> simpInt = SimpsonIntegrator<float>(dt);
    TrapezoidalIntegrator<float> trapInt = TrapezoidalIntegrator<float>(dt);

    float maxTime = M_PI;

    for(int _t=0; _t<(maxTime / dt + 1); _t++) {
        float t = (float)_t * dt;
        std::cout << "t " << t << std::endl;
        float val = sin(t);
        trapInt.step(val);
        simpInt.step(val);
        std::cout << "TRAP " << trapInt.result() << "\t" << "SIMP " << simpInt.result() << std::endl;
    }

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


template <typename T>
void SimpsonIntegrator<T>::step(T nextValue) {
    if(!hasFirstValue) {
        firstValue = nextValue;
        hasFirstValue = true;
    }
    else {
        if(sumState == 2) {
            aggregate += Integrator<T>::dt * (3 * values[0] + 3 * values[1]);
            if(currentStep != 3) {
                aggregate += Integrator<T>::dt * values[2];
            }

            values[2] = nextValue;
            aggregate += Integrator<T>::dt * values[2];

            sumState = 0;
        }
        else {
            values[sumState] = nextValue;
            sumState++;
        }
    }
    
    currentStep++;
}

template <typename T>
T SimpsonIntegrator<T>::result() {
    int rem = (currentStep - 1) % 3;
    T adjustment = 0;
    if(rem == 1) {
        // approximate the last part with trapezoidal
        adjustment = Integrator<T>::dt * (values[2] + values[0]) / 2;
    }
    else if(rem == 2) {
        // approximate the last part with 1/3 simpson's rule
        adjustment = Integrator<T>::dt * (values[2] + 4 * values[0] + values[1]) / 3;
    }
    return (3./8) * aggregate + adjustment;
}
