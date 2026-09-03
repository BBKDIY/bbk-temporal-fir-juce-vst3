#pragma once

#include <vector>

namespace bbk
{
    class PhaseFilterDesigner
    {
    public:
        enum class Target
        {
            minimumPhase,
            linearPhase
        };

        struct DesignResult
        {
            std::vector<float> impulse;
            int latencySamples = 0;
            int fftSize = 0;
        };

        static DesignResult design (double sampleRate, Target target);

    private:
        static int nextPowerOfTwo (int value);
    };
}
