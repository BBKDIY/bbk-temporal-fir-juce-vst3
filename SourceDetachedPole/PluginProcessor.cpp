#include "PluginProcessor.h"
#include "PluginEditor.h"

BBKDetachedPoleAudioProcessor::BBKDetachedPoleAudioProcessor()
: AudioProcessor (BusesProperties()
    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
    .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
  parameters (*this, nullptr, "PARAMETERS", createParameterLayout())
{
}

juce::AudioProcessorValueTreeState::ParameterLayout BBKDetachedPoleAudioProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    layout.add (std::make_unique<juce::AudioParameterChoice> (
        juce::ParameterID { "mode", 1 }, "Mode",
        juce::StringArray { "Bypass", "Case B: FIR only", "Case C: FIR + Pole" }, 0));
    return layout;
}

BBKDetachedPoleAudioProcessor::Mode BBKDetachedPoleAudioProcessor::getModeForUI() const noexcept
{
    if (const auto* value = parameters.getRawParameterValue ("mode"))
        return static_cast<Mode> (juce::jlimit (0, 2, static_cast<int> (std::lround (value->load()))));
    return Mode::bypass;
}

void BBKDetachedPoleAudioProcessor::prepareToPlay (double sampleRate, int)
{
    currentSampleRate.store (sampleRate);

    const bool valid = std::abs (sampleRate - static_cast<double> (bbk::detachedpole::sampleRateHz)) < 0.5;
    valid192k.store (valid);

    const int requiredChannels = juce::jmax (getTotalNumInputChannels(), getTotalNumOutputChannels());
    constexpr int channelHeadroom = 16;
    const int channelsToAllocate = juce::jmax (requiredChannels, channelHeadroom);

    const bool formatChanged = ! hasPrepared
                             || std::abs (sampleRate - lastPreparedSampleRate) > 0.5
                             || static_cast<int> (channels.size()) < channelsToAllocate;

    if (formatChanged)
    {
        channels.resize (static_cast<std::size_t> (channelsToAllocate));
        for (auto& channel : channels)
            channel.clear();

        modeCrossfade.reset (sampleRate, 0.015); // 15 ms click-free mode-select crossfade
        modeCrossfade.setCurrentAndTargetValue (0.0);
        fromMode = getModeForUI();
        toMode = fromMode;
        lastSeenMode = fromMode;
    }

    lastPreparedSampleRate = sampleRate;
    hasPrepared = true;

    // Case B own group delay is exactly 9 samples (linear phase, no
    // pole); Case C is flat to within +/-0.002 samples of ~9.515 across
    // 0-20 kHz. latencySamples (10) is used uniformly for all three modes
    // so the reported host latency never changes at runtime regardless of
    // which mode is selected - which is what matters for host PDC
    // correctness. See DetachedPoleFilter.h.
    setLatencySamples (bbk::detachedpole::latencySamples);
}

bool BBKDetachedPoleAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto input = layouts.getMainInputChannelSet();
    const auto output = layouts.getMainOutputChannelSet();

    if (input != output)
        return false;

    return output == juce::AudioChannelSet::mono()
        || output == juce::AudioChannelSet::stereo();
}

template <typename SampleType>
void BBKDetachedPoleAudioProcessor::process (juce::AudioBuffer<SampleType>& buffer)
{
    if (! valid192k.load())
        return; // hard safety bypass outside 192 kHz - "REQUIRES 192 kHz" is shown in the UI

    const int numChannels = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    if (static_cast<int> (channels.size()) < numChannels)
    {
        const auto oldSize = channels.size();
        channels.resize (static_cast<std::size_t> (numChannels));
        for (auto i = oldSize; i < channels.size(); ++i)
            channels[i].clear();
    }

    // Detect a mode change on the message/automation side and (re)start
    // the mode-select crossfade. If a new change arrives mid-crossfade,
    // the in-flight "from" mode keeps playing uninterrupted and only the
    // "to" mode and the ramp restart - this can never click, since the
    // audibly-playing signal never changes discontinuously.
    const Mode requestedMode = getModeForUI();
    if (requestedMode != lastSeenMode)
    {
        lastSeenMode = requestedMode;
        if (requestedMode != toMode)
        {
            toMode = requestedMode;
            if (toMode != fromMode)
            {
                modeCrossfade.setCurrentAndTargetValue (0.0);
                modeCrossfade.setTargetValue (1.0);
            }
        }
    }

    const bool transitioning = (fromMode != toMode);

    using namespace bbk::detachedpole;
    const auto& taps = firTaps();

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const double crossfadeAmount = transitioning ? modeCrossfade.getNextValue() : 1.0;

        for (int ch = 0; ch < numChannels; ++ch)
        {
            auto& state = channels[static_cast<std::size_t> (ch)];
            auto* data = buffer.getWritePointer (ch);
            const double x = static_cast<double> (data[sample]);

            state.history[static_cast<std::size_t> (state.writeIndex)] = x;

            // Stage 1: 19-tap linear-phase FIR - shared by Case B and Case C.
            double w = 0.0;
            for (int k = 0; k < firTapCount; ++k)
            {
                int index = state.writeIndex - k;
                if (index < 0)
                    index += historyLength;
                w += taps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
            }

            // Stage 2: first-order real-pole IIR, always updated from the
            // the FIR own continuous output so Case C own state stays
            // warm and a mode switch into it is never a cold-start
            // transient (see header comment).
            const double wetC = poleB0 * w + poleB1 * state.poleW1 - poleA1 * state.poleY1;
            state.poleW1 = w;
            state.poleY1 = wetC;

            const double wetB = w;

            int dryIndex = state.writeIndex - latencySamples;
            if (dryIndex < 0)
                dryIndex += historyLength;
            const double dry = state.history[static_cast<std::size_t> (dryIndex)];

            auto signalFor = [&] (Mode m) -> double
            {
                switch (m)
                {
                    case Mode::bypass: return dry;
                    case Mode::caseB:  return wetB;
                    case Mode::caseC:  return wetC;
                }
                return dry;
            };

            const double fromSignal = signalFor (fromMode);
            double out = fromSignal;
            if (transitioning)
            {
                const double toSignal = signalFor (toMode);
                out = fromSignal + crossfadeAmount * (toSignal - fromSignal);
            }

            data[sample] = static_cast<SampleType> (out);

            if (++state.writeIndex == historyLength)
                state.writeIndex = 0;
        }
    }

    if (transitioning && ! modeCrossfade.isSmoothing())
        fromMode = toMode;
}

void BBKDetachedPoleAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

void BBKDetachedPoleAudioProcessor::processBlock (juce::AudioBuffer<double>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

juce::AudioProcessorEditor* BBKDetachedPoleAudioProcessor::createEditor()
{
    return new BBKDetachedPoleAudioProcessorEditor (*this);
}

void BBKDetachedPoleAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = parameters.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void BBKDetachedPoleAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (parameters.state.getType()))
            parameters.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BBKDetachedPoleAudioProcessor();
}
