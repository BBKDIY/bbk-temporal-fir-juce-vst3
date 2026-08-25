#include "PluginProcessor.h"
#include "PluginEditor.h"

BBKDetachedPoleAudioProcessor::BBKDetachedPoleAudioProcessor()
: AudioProcessor (BusesProperties()
    .withInput ("Input", juce::AudioChannelSet::stereo(), true)
    .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
  juce::Thread ("BBKDetachedPole Design Thread"),
  parameters (*this, nullptr, "PARAMETERS", createParameterLayout())
{
    parameters.addParameterListener ("cutoff", &paramListener);
    parameters.addParameterListener ("attenuation", &paramListener);
    parameters.addParameterListener ("stopband", &paramListener);

    startThread();
}

BBKDetachedPoleAudioProcessor::~BBKDetachedPoleAudioProcessor()
{
    // Unregister before any members are destroyed - the listener list
    // inside `parameters` holds a raw pointer to paramListener, and
    // members are destroyed in reverse declaration order (paramListener
    // is declared right after parameters), so this must happen here in
    // the destructor body rather than being left implicit.
    parameters.removeParameterListener ("cutoff", &paramListener);
    parameters.removeParameterListener ("attenuation", &paramListener);
    parameters.removeParameterListener ("stopband", &paramListener);

    signalThreadShouldExit();
    notify();
    stopThread (5000);
}

juce::AudioProcessorValueTreeState::ParameterLayout BBKDetachedPoleAudioProcessor::createParameterLayout()
{
    using namespace bbk::detachedpole;
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "cutoff", 1 }, "Cutoff",
        juce::NormalisableRange<float> (static_cast<float> (minCutoffHz), static_cast<float> (maxCutoffHz), 1.0f),
        static_cast<float> (defaultCutoffHz)));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "attenuation", 1 }, "Attenuation at Cutoff",
        juce::NormalisableRange<float> (static_cast<float> (minAttenuationDb), static_cast<float> (maxAttenuationDb), 0.01f),
        static_cast<float> (defaultAttenuationDb)));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "stopband", 1 }, "Min. Stopband Rejection",
        juce::NormalisableRange<float> (static_cast<float> (minStopbandRejectionDb), static_cast<float> (maxStopbandRejectionDb), 0.1f),
        static_cast<float> (defaultStopbandRejectionDb)));

    return layout;
}

bbk::parametric::FilterSpec BBKDetachedPoleAudioProcessor::specFromParameters() const
{
    bbk::parametric::FilterSpec spec;

    const double sr = currentSampleRate.load();
    spec.sampleRateHz = sr > 0.0 ? sr : 192000.0;
    const double nyquist = spec.sampleRateHz * 0.5;

    const double cutoffParam = static_cast<double> (parameters.getRawParameterValue ("cutoff")->load());
    // Guard against a cutoff parameter above this sample rate's own
    // Nyquist (e.g. the 20 kHz default at 44.1 kHz, whose Nyquist is only
    // 22.05 kHz) - clamp well inside it so a real transition band always
    // has room to exist.
    spec.cutoffHz = juce::jmin (cutoffParam, nyquist * 0.9);
    spec.attenuationAtCutoffDb = static_cast<double> (parameters.getRawParameterValue ("attenuation")->load());
    spec.stopbandRejectionDb = static_cast<double> (parameters.getRawParameterValue ("stopband")->load());
    return spec;
}

void BBKDetachedPoleAudioProcessor::redesignSynchronously (const bbk::parametric::FilterSpec& spec)
{
    // Only ever called from prepareToPlay(), which the host guarantees is
    // never concurrent with processBlock() - safe to touch the
    // audio-thread-owned tap arrays and consumedVersion directly here.
    auto result = bbk::parametric::designParametricFIR (spec, bbk::detachedpole::maxTapCount);
    const auto padded = bbk::detachedpole::padTapsToFixedLength (result.taps);

    activeTaps = padded;
    incomingTaps = padded;
    crossfading = false;
    designCrossfade.setCurrentAndTargetValue (0.0);

    int newVersion;
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        newVersion = ++versionCounter;
        requestedSpec = spec;
        requestedVersion = newVersion;
    }
    {
        const juce::SpinLock::ScopedLockType sl (resultLock);
        latestResult = result;
        latestSpec = spec;
        latestVersion = newVersion;
    }
    consumedVersion = newVersion;

    {
        const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
        uiSnapshot.sampleRateHz = spec.sampleRateHz;
        uiSnapshot.cutoffHz = spec.cutoffHz;
        uiSnapshot.attenuationAtCutoffDb = spec.attenuationAtCutoffDb;
        uiSnapshot.stopbandRejectionDb = spec.stopbandRejectionDb;
        uiSnapshot.tapCount = result.tapCount;
        uiSnapshot.achievedStopbandDb = result.achievedStopbandDb;
        uiSnapshot.constraintsMet = result.constraintsMet;
        uiSnapshot.designAttempts = result.designAttempts;
        uiSnapshot.taps = result.taps;
    }
}

void BBKDetachedPoleAudioProcessor::requestBackgroundRedesign()
{
    // May be called from the message thread (typical - a slider moved) or
    // from the audio thread (a host delivered automation for one of these
    // parameters mid-block) - either way this only ever copies a small
    // FilterSpec under a SpinLock and signals the worker thread; the
    // actual (slow) design work never runs here.
    if (! hasPrepared.load() || currentSampleRate.load() <= 0.0)
        return;

    const auto spec = specFromParameters();
    {
        const juce::SpinLock::ScopedLockType sl (specLock);
        requestedSpec = spec;
        requestedVersion = ++versionCounter;
    }
    notify();
}

void BBKDetachedPoleAudioProcessor::run()
{
    while (! threadShouldExit())
    {
        wait (-1);
        if (threadShouldExit())
            break;

        bbk::parametric::FilterSpec specToRun;
        int versionToRun;
        {
            const juce::SpinLock::ScopedLockType sl (specLock);
            specToRun = requestedSpec;
            versionToRun = requestedVersion;
        }

        auto result = bbk::parametric::designParametricFIR (specToRun, bbk::detachedpole::maxTapCount);

        // Only publish if this is still the newest request - a stale
        // in-flight design finishing after a newer one was already
        // requested (or after redesignSynchronously ran on the message
        // thread while this was mid-design) must never overwrite a newer
        // result. Since this worker only ever runs one design at a time
        // and always re-reads the latest request before starting the next
        // one, the only way to see a stale version here is that race.
        bool isNewest = false;
        {
            const juce::SpinLock::ScopedLockType sl (resultLock);
            if (versionToRun > latestVersion)
            {
                latestResult = result;
                latestSpec = specToRun;
                latestVersion = versionToRun;
                isNewest = true;
            }
        }

        if (isNewest)
        {
            const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
            uiSnapshot.sampleRateHz = specToRun.sampleRateHz;
            uiSnapshot.cutoffHz = specToRun.cutoffHz;
            uiSnapshot.attenuationAtCutoffDb = specToRun.attenuationAtCutoffDb;
            uiSnapshot.stopbandRejectionDb = specToRun.stopbandRejectionDb;
            uiSnapshot.tapCount = result.tapCount;
            uiSnapshot.achievedStopbandDb = result.achievedStopbandDb;
            uiSnapshot.constraintsMet = result.constraintsMet;
            uiSnapshot.designAttempts = result.designAttempts;
            uiSnapshot.taps = result.taps;
        }
    }
}

BBKDetachedPoleAudioProcessor::DesignSnapshot BBKDetachedPoleAudioProcessor::getDesignSnapshotForUI() const
{
    const juce::SpinLock::ScopedLockType sl (uiSnapshotLock);
    return uiSnapshot;
}

void BBKDetachedPoleAudioProcessor::prepareToPlay (double sampleRate, int)
{
    currentSampleRate.store (sampleRate);

    const int requiredChannels = juce::jmax (getTotalNumInputChannels(), getTotalNumOutputChannels());
    constexpr int channelHeadroom = 16;
    const int channelsToAllocate = juce::jmax (requiredChannels, channelHeadroom);

    const bool formatChanged = ! hasPrepared.load()
                             || std::abs (sampleRate - lastPreparedSampleRate) > 0.5
                             || static_cast<int> (channels.size()) < channelsToAllocate;

    if (formatChanged)
    {
        channels.resize (static_cast<std::size_t> (channelsToAllocate));
        for (auto& channel : channels)
            channel.clear();

        designCrossfade.reset (sampleRate, 0.015); // 15 ms click-free redesign crossfade
        crossfading = false;

        // A sample-rate (or first-ever) change is a natural discontinuity
        // anyway - the host already expects a pause here - so the initial
        // design for the new rate is computed synchronously rather than
        // handed to the background thread, and applied immediately with
        // no crossfade (there is nothing meaningful to crossfade from: the
        // history buffers were just cleared above).
        redesignSynchronously (specFromParameters());
    }

    lastPreparedSampleRate = sampleRate;
    hasPrepared.store (true);

    // maxHalfLength samples, always - see DetachedPoleFilter.h. Every
    // design is zero-padded to this same fixed length, so the
    // host-reported latency never changes at runtime regardless of slider
    // values or sample rate.
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
    using namespace bbk::detachedpole;

    const int numChannels = buffer.getNumChannels();
    const int numSamples = buffer.getNumSamples();

    if (static_cast<int> (channels.size()) < numChannels)
    {
        const auto oldSize = channels.size();
        channels.resize (static_cast<std::size_t> (numChannels));
        for (auto i = oldSize; i < channels.size(); ++i)
            channels[i].clear();
    }

    // Best-effort, non-blocking pickup of a newer background design. If
    // the worker thread happens to be mid-publish this block simply
    // misses it and picks it up on the next block - never worth blocking
    // the audio thread for.
    {
        const juce::SpinLock::ScopedTryLockType tl (resultLock);
        if (tl.isLocked() && latestVersion != consumedVersion)
        {
            incomingTaps = padTapsToFixedLength (latestResult.taps);
            consumedVersion = latestVersion;
            crossfading = true;
            designCrossfade.setCurrentAndTargetValue (0.0);
            designCrossfade.setTargetValue (1.0);
        }
    }

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const double crossfadeAmount = crossfading ? designCrossfade.getNextValue() : 1.0;

        for (int ch = 0; ch < numChannels; ++ch)
        {
            auto& state = channels[static_cast<std::size_t> (ch)];
            auto* data = buffer.getWritePointer (ch);
            const double x = static_cast<double> (data[sample]);

            state.history[static_cast<std::size_t> (state.writeIndex)] = x;

            double wOld = 0.0;
            for (int k = 0; k < maxTapCount; ++k)
            {
                int index = state.writeIndex - k;
                if (index < 0) index += historyLength;
                wOld += activeTaps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
            }

            double out = wOld;
            if (crossfading)
            {
                double wNew = 0.0;
                for (int k = 0; k < maxTapCount; ++k)
                {
                    int index = state.writeIndex - k;
                    if (index < 0) index += historyLength;
                    wNew += incomingTaps[static_cast<std::size_t> (k)] * state.history[static_cast<std::size_t> (index)];
                }
                out = wOld + crossfadeAmount * (wNew - wOld);
            }

            data[sample] = static_cast<SampleType> (out);

            if (++state.writeIndex == historyLength)
                state.writeIndex = 0;
        }
    }

    if (crossfading && ! designCrossfade.isSmoothing())
    {
        activeTaps = incomingTaps;
        crossfading = false;
    }
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
