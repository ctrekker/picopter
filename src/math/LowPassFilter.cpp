#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(int size, float initVal) {
    this->historySize = size;
    this->history = new float[size];
    currentIndex = 0;

    for(int i=0; i<size; i++) {
        history[i] = initVal;
    }
}

LowPassFilter::~LowPassFilter() {
    delete[] this->history;
}


float LowPassFilter::step(float val) {
    history[currentIndex] = val;
    currentIndex = (currentIndex + 1) % historySize;

    float mean = 0.;
    for(int i=0; i<historySize; i++) {
        mean += history[i];
    }
    mean /= historySize;

    return mean;
}
