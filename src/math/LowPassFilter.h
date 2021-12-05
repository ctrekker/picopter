#pragma once

class LowPassFilter {
    private:
        float *history;
        int historySize;
        int currentIndex;

    public:
        LowPassFilter(int size, float initVal=0.);
        ~LowPassFilter();

        float step(float val);
};
