#include "PluginProcessor.h"
#include "PluginEditor.h"

BBKTemporalFIRAudioProcessor::BBKTemporalFIRAudioProcessor()
: AudioProcessor (BusesProperties()
    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
    .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
  parameters (*this, nullptr, "PARAMETERS", createParameterLayout())
{
    fromFilterIndex = defaultIndexForBaseline();
    toFilterIndex = fromFilterIndex;
    lastSeenParamIndex = fromFilterIndex;
}

int BBKTemporalFIRAudioProcessor::defaultIndexForBaseline() noexcept
{
    // The 19-tap design is the required baseline/default selection.
    const auto& bank = bbk::temporalfir::filterBank();
    for (int i = 0; i < bbk::temporalfir::numFilterPoints; ++i)
        if (bank[static_cast<std::size_t> (i)].trueTapCount == 19)
            return i;
    return 0;
}

juce::AudioProcessorValueTreeState::ParameterLayout BBKTemporalFIRAudioProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "enabled", 1 }, "Temporal FIR enabled", false));
    layout.add (std::make_unique<juce::AudioParameterInt> (
        juce::ParameterID { "tradeoffIndex", 1 }, "Temporal trade-off",
        0, bbk::temporalfir::numFilterPoints - 1, defaultIndexForBaseline()));
    return layout;
}

void BBKTemporalFIRAudioProcessor::prepareToPlay (double sampleRate, int)
{
    currentSampleRate.store (sampleRate);

    const bool valid = std::abs (sampleRate - static_cast<double> (bbk::temporalfir::sampleRateHz)) < 0.5;
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

        wetMix.reset (sampleRate, 0.010); // 10 ms click-free bypass/active crossfade
        const bool requested = isEnabledForUI();
        wetMix.setCurrentAndTargetValue ((valid && requested) ? 1.0 : 0.0);

        filterCrossfade.reset (sampleRate, 0.015); // 15 ms click-free filter-select crossfade
        filterCrossfade.setCurrentAndTargetValue (0.0);
        fromFilterIndex = getSelectedIndexForUI();
        toFilterIndex = fromFilterIndex;
        lastSeenParamIndex = fromFilterIndex;
    }

    lastPreparedSampleRate = sampleRate;
    hasPrepared = true;

    // Every selectable filter (and bypass) is delay-matched to exactly
    // maxGroupDelaySamples by construction (see TemporalFIRBank.h), so this
    // is safe to report as a constant regardless of which filter is
    // selected or whether the plugin is bypassed - it never changes at
    // runtime, which is the case that matters for host PDC correctness.
    setLatencySamples (bbk::temporalfir::maxGroupDelaySamples);
}

bool BBKTemporalFIRAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto input = layouts.getMainInputChannelSet();
    const auto output = layouts.getMainOutputChannelSet();

    if (input != output)
        return false;

    return output == juce::AudioChannelSet::mono()
        || output == juce::AudioChannelSet::stereo();
}

bool BBKTemporalFIRAudioProcessor::isEnabledForUI() const noexcept
{
    if (const auto* value = parameters.getRawParameterValue ("enabled"))
        return value->load() >= 0.5f;
    return false;
}

int BBKTemporalFIRAudioProcessor::getSelectedIndexForUI() const noexcept
{
    if (const auto* value = parameters.getRawParameterValue ("tradeoffIndex"))
        return juce::jlimit (0, bbk::temporalfir::numFilterPoints - 1,
                              static_cast<int> (std::lround (value->load())));
    return 0;
}

template <typename SampleType>
void BBKTemporalFIRAudioProcessor::process (juce::AudioBuffer<SampleType>& buffer)
{
    if (! valid192k.load())
        return; // hard safety bypass outside 192 kHz - "REQUIRES 192 kHz" is shown in the UI

    const bool enabled = isEnabledForUI();
    wetMix.setTargetValue (enabled ? 1.0 : 0.0);

    if (! enabled && wetMix.getCurrentValue() <= 0.0)
        return;

    // Detect a slider change on the message/automation side and (re)start
    // the filter-select crossfade. If a new change arrives mid-crossfade,
    // the in-flight "from" filter keeps playing uninterrupted and only the
    // "to" filter and the ramp restart - this can never click, since the
    // audibly-playing filter never changes discontinuously.
    const int requestedIndex = getSelectedIndexForUI();
    if (requestedIndex != lastSeenParamIndex)
    {
        lastSeenParamIndex = requestedIndex;
        if (requestedIndex != toFilterIndex)
        {
            toFilterIndex = requestedIndex;
            if (toFilterIndex != fromFilterIndex)
            {
                filterCrossfade.setCurrentAndTargetValue (0.0);
                filterCrossfade.setTargetValue (1.0);
            }
        }
    }

    const bool transitioning = (fromFilterIndex != toFilterIndex);
    const auto& fromPoint = filterPointFor (fromFilterIndex);
    const auto& toPoint = transitioning ? filterPointFor (toFilterIndex) : fromPoint;

    const int numChannels = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    if (static_cast<int> (channels.size()) < numChannels)
    {
        const auto oldSize = channels.size();
        channels.resize (static_cast<std::size_t> (numChannels));
        for (auto i = oldSize; i < channels.size(); ++i)
            channels[i].clear();
    }

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const double enabledAmount = wetMix.getNextValue();
        const double filterMixAmount = transitioning ? filterCrossfade.getNextValue() : 1.0;

        for (int ch = 0; ch < numChannels; ++ch)
        {
            auto& state = channels[static_cast<std::size_t> (ch)];
            auto* data = buffer.getWritePointer (ch);
            const double x = static_cast<double> (data[sample]);

            state.history[static_cast<std::size_t> (state.writeIndex)] = x;

            double wetFrom = 0.0;
            for (int k = 0; k < bbk::temporalfir::maxN; ++k)
            {
                int index = state.writeIndex - k;
                if (index < 0)
                    index += bbk::temporalfir::maxN;

                wetFrom += fromPoint.taps[static_cast<std::size_t> (k)]
                         * state.history[static_cast<std::size_t> (index)];
            }

            double wet = wetFrom;
            if (transitioning)
            {
                double wetTo = 0.0;
                for (int k = 0; k < bbk::temporalfir::maxN; ++k)
                {
                    int index = state.writeIndex - k;
                    if (index < 0)
                        index += bbk::temporalfir::maxN;

                    wetTo += toPoint.taps[static_cast<std::size_t> (k)]
                           * state.history[static_cast<std::size_t> (index)];
                }
                wet = wetFrom + filterMixAmount * (wetTo - wetFrom);
            }

            int dryIndex = state.writeIndex - bbk::temporalfir::maxGroupDelaySamples;
            if (dryIndex < 0)
                dryIndex += bbk::temporalfir::maxN;

            const double dry = state.history[static_cast<std::size_t> (dryIndex)];
            const double y = dry + enabledAmount * (wet - dry);
            data[sample] = static_cast<SampleType> (y);

            if (++state.writeIndex == bbk::temporalfir::maxN)
                state.writeIndex = 0;
        }
    }

    if (transitioning && ! filterCrossfade.isSmoothing())
        fromFilterIndex = toFilterIndex;
}

void BBKTemporalFIRAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

void BBKTemporalFIRAudioProcessor::processBlock (juce::AudioBuffer<double>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    process (buffer);
}

juce::AudioProcessorEditor* BBKTemporalFIRAudioProcessor::createEditor()
{
    return new BBKTemporalFIRAudioProcessorEditor (*this);
}

void BBKTemporalFIRAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = parameters.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void BBKTemporalFIRAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (parameters.state.getType()))
            parameters.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BBKTemporalFIRAudioProcessor();
}
