#ifndef PHASESIGNALGENERATOR_H
#define PHASESIGNALGENERATOR_H

#include "macros.h"
#include "a1.h"

#include <algorithm>
#include <iterator>
#include <iostream>

class PhaseSignalGenerator {
    public:
        typedef unsigned long int Time;
        static inline Time now() { return time_us(); }

    private:
        A1 *a1_base_;
        Time last_touchdown_;
        bool has_swung_;

        // Parameter
        float stance_duration_ = 0.25;

    public:
        bool has_started;
        float stance_phase_signal[4];
        float swing_phase_signal[4];

        PhaseSignalGenerator(A1 &a1_base, Time time = now()):
            a1_base_(&a1_base),
            last_touchdown_(time),
            has_swung_(false),
            has_started(false),
            stance_phase_signal{0.0f, 0.0f, 0.0f, 0.0f},
            swing_phase_signal{0.0f, 0.0f, 0.0f, 0.0f}
        {
        }

        void run(float velocity, float step_length, Time time);

};

#endif