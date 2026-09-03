#include "../Source/PhaseFilterDesigner.h"
#include <fstream>
#include <string>

int main()
{
    for (const auto pair : { std::pair { bbk::PhaseFilterDesigner::Target::minimumPhase, std::string ("min48.f32") },
                             std::pair { bbk::PhaseFilterDesigner::Target::linearPhase, std::string ("linear48.f32") } })
    {
        const auto r = bbk::PhaseFilterDesigner::design (48000.0, pair.first);
        std::ofstream out (pair.second, std::ios::binary);
        out.write (reinterpret_cast<const char*> (r.impulse.data()),
                   static_cast<std::streamsize> (r.impulse.size() * sizeof (float)));
    }
}
