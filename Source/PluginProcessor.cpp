#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <cmath>

namespace
{
    // Every filter in bbk::temporalfir::filterBank() has unity DC gain (a
    // steady tone passes through at the same level), but that does NOT mean
    // unity PEAK gain: each filter also has small negative sidelobes, and
    // for any filter with negative taps, sum(|taps|) is strictly greater
    // than sum(taps) = 1. That gap - the L1 norm of the impulse response -
    // is the exact worst-case peak gain the filter can apply to any
    // bounded input. Measured directly from the real coefficients in
    // TemporalFIRBank.h, this ranges from about +2.55 dB (shortest filter)
    // to +4.24 dB (longest/steepest) depending which "Sidelobe Decay"
    // point is selected. Real program material lands far below that
    // theoretical ceiling in practice (empirically ~+0.15 to +0.5 dB over
    // in Monte Carlo testing against the actual filters) - but modern
    // masters are routinely mixed to within a few tenths of a dB of
    // 0 dBFS already, so even that modest, realistic overshoot is enough
    // to clip audibly on the loudest passages, which is exactly what was
    // reported. Unlike BBK Phase Corrector's allpass (worst case in the
    // tens of dB), the ceiling here is small enough that a fixed pad CAN
    // give a complete, mathematically airtight guarantee without
    // meaningfully compromising the output level.
    //
    // Applied only to the WET (filtered) signal, before it's mixed with
    // dry - "disabled"/BYPASS stays byte-for-byte untouched regardless of
    // this setting, same transparency principle as BBK Phase Corrector.
    constexpr float softClipKneeStart = 0.891f; // ~ -1 dBFS: identity below this
    constexpr float softClipCeiling   = 0.999f; // ~ -0.01 dBFS: asymptote, never reached exactly

    inline double applySafetySoftClip (double x) noexcept
    {
        const double ax = std::abs (x);
        if (ax <= softClipKneeStart)
            return x;

        const double span = static_cast<double> (softClipCeiling) - softClipKneeStart;
        const double over = (ax - softClipKneeStart) / span;
        const double shaped = softClipKneeStart + span * std::tanh (over);
        return std::copysign (shaped, x);
    }

    // Auto-headroom calibration tuning - one-way ratchet only, same design
    // as BBK Phase Corrector. Smaller step and range than Phase Corrector
    // since the whole problem here is only a few dB, not tens.
    constexpr float autoHeadroomStepDb           = 0.25f;
    constexpr float autoHeadroomMinDb            = -6.0f; // matches the "headroom" parameter's range floor
    constexpr float autoHeadroomTriggerLinear    = 0.01f; // ~0.1 dB sustained excess over the knee before ratcheting
    constexpr float autoHeadroomReleasePerSecond = 0.5f;
    constexpr double autoHeadroomCooldownSeconds = 3.0;
}

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

    // See the anonymous namespace above process() for the full rationale:
    // every filter here has unity DC gain but not unity peak gain, so the
    // WET signal alone can exceed 0 dBFS on already-loud material. -1 dB
    // default comfortably covers the empirically-measured realistic worst
    // case (~+0.5 dB); the range floor of -6 dB comfortably covers the
    // exact theoretical worst case for every filter in the bank
    // (+2.55 to +4.24 dB) with margin.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "headroom", 1 },
        "Headroom",
        juce::NormalisableRange<float> (-6.0f, 0.0f, 0.1f),
        -1.0f,
        juce::AudioParameterFloatAttributes().withLabel ("dB")));

    // One-way ratchet, default on - see the auto-headroom constants above
    // process() and PluginEditor.cpp for how manual edits turn this off.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "autoHeadroom", 1 },
        "Auto Headroom",
        true));

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

    clipEnvelope = 0.0f;
    samplesUntilNextAutoAdjust = 0;
    autoAdjustCooldownSamples = static_cast<int> (sampleRate * autoHeadroomCooldownSeconds);

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

    // See the anonymous namespace above for the full rationale. Read once
    // per block, not per sample - this only needs to react on the order of
    // seconds, not samples.
    const float headroomDb = parameters.getRawParameterValue ("headroom")->load();
    const double preAttenuationGain = juce::Decibels::decibelsToGain (headroomDb);
    const bool autoHeadroomEnabled = parameters.getRawParameterValue ("autoHeadroom")->load() > 0.5f;
    double blockPeakOver = 0.0;

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

            // Pad + soft-clip backstop applied to the WET signal only -
            // "disabled"/BYPASS (enabledAmount == 0, so y == dry exactly)
            // is completely untouched by either, same transparency
            // principle as BBK Phase Corrector. See the anonymous
            // namespace above process() for why this is needed at all.
            wet *= preAttenuationGain;

            const double absWet = std::abs (wet);
            if (absWet > softClipKneeStart)
                blockPeakOver = std::max (blockPeakOver, absWet - static_cast<double> (softClipKneeStart));

            wet = applySafetySoftClip (wet);

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

    // Auto-headroom: one-way ratchet only, same design as BBK Phase
    // Corrector's Auto Headroom - see the tuning constants above and the
    // member comments in PluginProcessor.h.
    if (autoHeadroomEnabled && enabled)
    {
        const auto sampleRate = currentSampleRate.load();
        if (sampleRate > 0.0)
        {
            const float releaseThisBlock = autoHeadroomReleasePerSecond
                                          * static_cast<float> (numSamples)
                                          / static_cast<float> (sampleRate);
            clipEnvelope = std::max (static_cast<float> (blockPeakOver), clipEnvelope - releaseThisBlock);

            samplesUntilNextAutoAdjust -= numSamples;

            if (clipEnvelope > autoHeadroomTriggerLinear && samplesUntilNextAutoAdjust <= 0)
            {
                if (auto* headroomParam = parameters.getParameter ("headroom"))
                {
                    const float newDb = std::max (autoHeadroomMinDb, headroomDb - autoHeadroomStepDb);
                    if (newDb != headroomDb)
                        headroomParam->setValueNotifyingHost (headroomParam->convertTo0to1 (newDb));
                }

                clipEnvelope = 0.0f;
                samplesUntilNextAutoAdjust = autoAdjustCooldownSamples;
            }
        }
    }
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
