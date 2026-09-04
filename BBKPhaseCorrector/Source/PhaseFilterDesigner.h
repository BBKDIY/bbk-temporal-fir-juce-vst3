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

        // correctionDepth scales the final correction curve uniformly (0 =
        // no correction at all, i.e. a pure delay identical in effect to
        // BYPASS but latency-matched; 1 = the full measured correction).
        // Scaling the angle rather than picking a different measurement
        // keeps every frequency's natural zero-crossing exactly where it
        // is and just shrinks the swing around it - see PhaseFilterDesigner.cpp.
        static DesignResult design (double sampleRate, Target target, float correctionDepth = 1.0f);

    private:
        static int nextPowerOfTwo (int value);
    };
}
