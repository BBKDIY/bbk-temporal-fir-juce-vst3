#include "../Source/PhaseFilterDesigner.h"

#include <algorithm>
#include <cmath>
#include <iostream>

int main()
{
    for (const auto rate : { 44100.0, 48000.0, 96000.0, 192000.0 })
    {
        for (const auto target : { bbk::PhaseFilterDesigner::Target::minimumPhase,
                                   bbk::PhaseFilterDesigner::Target::linearPhase })
        {
            const auto result = bbk::PhaseFilterDesigner::design (rate, target);
            double energy = 0.0;
            float peak = 0.0f;
            for (const auto x : result.impulse)
            {
                energy += static_cast<double> (x) * x;
                peak = std::max (peak, std::abs (x));
            }

            std::cout << rate << " Hz  "
                      << (target == bbk::PhaseFilterDesigner::Target::minimumPhase ? "minimum" : "linear")
                      << "  taps=" << result.fftSize
                      << "  latency=" << result.latencySamples
                      << "  energy=" << energy
                      << "  peak=" << peak << '\n';
        }
    }
}
