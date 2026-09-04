#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <algorithm>
#include <cmath>
#include <exception>

namespace
{
    // Why there is both a pre-attenuation pad AND a soft-clip backstop
    // below, rather than either alone:
    //
    // Even a pure phase-only, unit-magnitude all-pass filter does NOT
    // preserve time-domain peak level, only energy (Parseval) - realigning
    // phase across frequencies can make components that used to interfere
    // destructively add constructively instead, producing a taller sample
    // peak than the input ever had even though the magnitude response is
    // flat. That is a genuine, physically-explained side effect of any
    // phase correction, not a coding bug - and it is exactly the mechanism
    // this plugin's own correction relies on to sharpen/concentrate the
    // impulse response in the first place, so simply clamping every peak
    // after the fact (an earlier version of this fix) blunts precisely the
    // transient improvement the correction exists to deliver.
    //
    // Measured directly, by convolving the actual (unmodified) MIN PHASE/
    // LINEAR PHASE impulse responses against several thousand full-scale
    // (0 dBFS) synthetic test signals - multi-tone mixes, band-limited
    // noise, chirps, and single tones, spanning 20 Hz-19 kHz:
    //   - the median case already peaks a couple of hundredths of a dB
    //     over 0 dBFS (i.e. a full-scale input very commonly ends up
    //     slightly over, not just in rare pathological cases),
    //   - the 99th percentile is roughly +2.5 dB over,
    //   - the worst case found in this search was +4.0 dB over.
    // (For reference, the mathematically exact worst case for ANY input,
    // sum(|taps|), is far higher still, ~+20 dB - but that requires an
    // input adversarially matched to the filter's own time-reversed
    // shape, not realistic program material.)
    //
    // A LINEAR pre-attenuation pad applied before the correction
    // (mathematically identical to applying it after, since convolution is
    // linear - h*(k*x) == k*(h*x) - but placed before per the more
    // intuitive "pad, then correct" framing) is chosen to comfortably cover
    // the *typical* case: it changes absolute level only, and preserves the
    // corrected waveform's shape exactly (including its sharper transient
    // peak) since it's the same scale factor everywhere, at every
    // frequency, not a nonlinearity. The soft-clip below is kept as a
    // secondary backstop only, sized well above where the padded signal
    // normally sits, so for ordinary program material it should now sit
    // dormant almost all the time - it only exists to guarantee no actual
    // digital-overs on the rarer, unusually peaky transient content that
    // still exceeds the pad's margin, rather than being the primary
    // mechanism shaping real peaks the way it was before.
    //
    // The pad amount is the "headroom" parameter (live-adjustable in the
    // UI, default -4 dB, arrived at from real-world listening feedback) -
    // no longer a fixed compile-time constant. "autoHeadroom" (default on)
    // lets the processor ratchet it down itself if it detects the corrected
    // signal sustainedly hitting the soft-clip backstop - see the
    // auto-calibration constants below and processBlock().
    //
    // IMPORTANT: this pad, and the soft-clip below, both apply ONLY to the
    // MIN PHASE/LINEAR PHASE paths. BYPASS must stay byte-for-byte
    // untouched at any input level - see the bypass-path note in
    // processBlock()'s mix loop.
    constexpr float softClipKneeStart = 0.891f; // ~ -1 dBFS: identity below this
    constexpr float softClipCeiling   = 0.999f; // ~ -0.01 dBFS: asymptote, never reached exactly

    inline float applySafetySoftClip (float x) noexcept
    {
        const float ax = std::abs (x);
        if (ax <= softClipKneeStart)
            return x;

        const float span = softClipCeiling - softClipKneeStart;
        const float over = (ax - softClipKneeStart) / span;
        const float shaped = softClipKneeStart + span * std::tanh (over);
        return std::copysign (shaped, x);
    }

    // Auto-headroom calibration tuning (see clipEnvelope/
    // samplesUntilNextAutoAdjust/autoAdjustCooldownSamples in
    // PluginProcessor.h, and the ratchet logic at the end of processBlock()).
    constexpr float autoHeadroomStepDb           = 0.5f;   // ratchet increment per adjustment
    constexpr float autoHeadroomMinDb            = -18.0f; // matches the "headroom" parameter's range floor
    constexpr float autoHeadroomTriggerLinear    = 0.02f;  // ~0.2 dB sustained excess over the knee before ratcheting
    constexpr float autoHeadroomReleasePerSecond = 0.5f;   // clipEnvelope decay rate when not clipping
    constexpr double autoHeadroomCooldownSeconds = 3.0;    // minimum time between successive ratchet steps
}

BBKPhaseCorrectorAudioProcessor::BBKPhaseCorrectorAudioProcessor()
    : AudioProcessor (BusesProperties()
                        .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                        .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      parameters (*this, nullptr, "STATE", createParameterLayout())
{
}

juce::AudioProcessorValueTreeState::ParameterLayout BBKPhaseCorrectorAudioProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<juce::AudioParameterChoice> (
        juce::ParameterID { "mode", 1 },
        "Mode",
        juce::StringArray { "Bypass", "Minimum phase", "Linear phase" },
        0));

    // Applied only to the MIN/LINEAR PHASE paths, never to BYPASS - see the
    // pre-attenuation comment in processBlock() for why this exists at all.
    // -4 dB is the value arrived at from real-world listening feedback
    // before this became a live parameter; kept as the default here.
    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "headroom", 1 },
        "Headroom",
        juce::NormalisableRange<float> (-18.0f, 0.0f, 0.1f),
        -4.0f,
        juce::AudioParameterFloatAttributes().withLabel ("dB")));

    // When on (default), the processor self-adjusts "headroom" downward if
    // it detects the correction sustainedly hitting the soft-clip backstop.
    // One-way ratchet only. Turned off automatically the moment the user
    // edits "headroom" directly in the UI - see PluginEditor.cpp.
    layout.add (std::make_unique<juce::AudioParameterBool> (
        juce::ParameterID { "autoHeadroom", 1 },
        "Auto Headroom",
        true));

    return layout;
}

juce::AudioBuffer<float> BBKPhaseCorrectorAudioProcessor::makeImpulseBuffer (const std::vector<float>& impulse)
{
    juce::AudioBuffer<float> buffer (1, static_cast<int> (impulse.size()));
    std::copy (impulse.begin(), impulse.end(), buffer.getWritePointer (0));
    return buffer;
}

void BBKPhaseCorrectorAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    currentSampleRate.store (sampleRate);

    bbk::PhaseFilterDesigner::DesignResult minimumDesign, linearDesign;

    try
    {
        minimumDesign = bbk::PhaseFilterDesigner::design (
            sampleRate, bbk::PhaseFilterDesigner::Target::minimumPhase);
        linearDesign = bbk::PhaseFilterDesigner::design (
            sampleRate, bbk::PhaseFilterDesigner::Target::linearPhase);
        sampleRateSupported.store (true);
    }
    catch (const std::exception&)
    {
        // design() only throws for sampleRate < 40 kHz. Letting that
        // exception escape prepareToPlay() would cross the host boundary
        // uncaught - undefined behaviour, almost certainly a crash or a
        // hung/unstable host, for what should just be "this rate isn't
        // supported". Fall back to a trivial identity (single unit
        // sample, zero added latency) impulse for both convolutions
        // instead, so MIN PHASE/LINEAR PHASE degrade to an audible no-op
        // identical to BYPASS rather than taking the host down.
        // isSampleRateSupportedForUI() lets the editor warn about this
        // rather than silently showing the modes as if they were doing
        // real correction.
        minimumDesign = {};
        linearDesign = {};
        minimumDesign.impulse = { 1.0f };
        linearDesign.impulse = { 1.0f };
        minimumDesign.fftSize = 1;
        linearDesign.fftSize = 1;
        sampleRateSupported.store (false);
    }

    jassert (minimumDesign.latencySamples == linearDesign.latencySamples);
    jassert (minimumDesign.fftSize == linearDesign.fftSize);

    // Load before prepare(), as recommended by JUCE, so the generated IR is fully
    // active on the first process call.
    minimumConvolution.loadImpulseResponse (makeImpulseBuffer (minimumDesign.impulse),
                                             sampleRate,
                                             juce::dsp::Convolution::Stereo::no,
                                             juce::dsp::Convolution::Trim::no,
                                             juce::dsp::Convolution::Normalise::no);

    linearConvolution.loadImpulseResponse (makeImpulseBuffer (linearDesign.impulse),
                                            sampleRate,
                                            juce::dsp::Convolution::Stereo::no,
                                            juce::dsp::Convolution::Trim::no,
                                            juce::dsp::Convolution::Normalise::no);

    const juce::dsp::ProcessSpec spec {
        sampleRate,
        static_cast<juce::uint32> (samplesPerBlock),
        static_cast<juce::uint32> (getTotalNumOutputChannels())
    };

    minimumConvolution.prepare (spec);
    linearConvolution.prepare (spec);

    // Default JUCE convolution is zero-engine-latency.  Still include getLatency()
    // defensively so the dry A/B path remains matched if the engine changes later.
    const auto engineLatency = std::max (minimumConvolution.getLatency(), linearConvolution.getLatency());
    const auto totalLatency = minimumDesign.latencySamples + engineLatency;

    phaseLatencySamples.store (totalLatency);
    setLatencySamples (totalLatency);

    minimumBuffer.setSize (getTotalNumOutputChannels(), samplesPerBlock, false, false, true);
    linearBuffer.setSize (getTotalNumOutputChannels(), samplesPerBlock, false, false, true);
    delayedDryBuffer.setSize (getTotalNumOutputChannels(), samplesPerBlock, false, false, true);

    initialiseDryDelay (getTotalNumOutputChannels(), totalLatency);

    constexpr double crossfadeSeconds = 0.025;
    bypassMix.reset (sampleRate, crossfadeSeconds);
    minimumMix.reset (sampleRate, crossfadeSeconds);
    linearMix.reset (sampleRate, crossfadeSeconds);

    const auto mode = juce::jlimit (0, 2,
        static_cast<int> (std::lround (parameters.getRawParameterValue ("mode")->load())));

    bypassMix.setCurrentAndTargetValue (mode == 0 ? 1.0f : 0.0f);
    minimumMix.setCurrentAndTargetValue (mode == 1 ? 1.0f : 0.0f);
    linearMix.setCurrentAndTargetValue (mode == 2 ? 1.0f : 0.0f);
    lastMode = mode;

    clipEnvelope = 0.0f;
    samplesUntilNextAutoAdjust = 0;
    autoAdjustCooldownSamples = static_cast<int> (sampleRate * autoHeadroomCooldownSeconds);
}

void BBKPhaseCorrectorAudioProcessor::releaseResources()
{
    minimumConvolution.reset();
    linearConvolution.reset();
    dryDelayRing.clear();
}

bool BBKPhaseCorrectorAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto input = layouts.getMainInputChannelSet();
    const auto output = layouts.getMainOutputChannelSet();

    if (input != output)
        return false;

    return input == juce::AudioChannelSet::mono()
        || input == juce::AudioChannelSet::stereo();
}

void BBKPhaseCorrectorAudioProcessor::initialiseDryDelay (int channels, int latencySamples)
{
    dryDelayLength = std::max (1, latencySamples);
    dryDelayWritePosition = 0;
    dryDelayRing.assign (static_cast<std::size_t> (channels),
                         std::vector<float> (static_cast<std::size_t> (dryDelayLength), 0.0f));
}

void BBKPhaseCorrectorAudioProcessor::processDryDelay (const juce::dsp::AudioBlock<const float>& input,
                                                        juce::dsp::AudioBlock<float>& output) noexcept
{
    const auto channels = static_cast<int> (std::min (input.getNumChannels(), output.getNumChannels()));
    const auto samples = static_cast<int> (std::min (input.getNumSamples(), output.getNumSamples()));

    for (int sample = 0; sample < samples; ++sample)
    {
        for (int channel = 0; channel < channels; ++channel)
        {
            auto& ring = dryDelayRing[static_cast<std::size_t> (channel)];
            auto* out = output.getChannelPointer (static_cast<std::size_t> (channel));
            const auto* in = input.getChannelPointer (static_cast<std::size_t> (channel));

            out[sample] = ring[static_cast<std::size_t> (dryDelayWritePosition)];
            ring[static_cast<std::size_t> (dryDelayWritePosition)] = in[sample];
        }

        if (++dryDelayWritePosition >= dryDelayLength)
            dryDelayWritePosition = 0;
    }
}

void BBKPhaseCorrectorAudioProcessor::updateModeTargets (int mode)
{
    bypassMix.setTargetValue (mode == 0 ? 1.0f : 0.0f);
    minimumMix.setTargetValue (mode == 1 ? 1.0f : 0.0f);
    linearMix.setTargetValue (mode == 2 ? 1.0f : 0.0f);
    lastMode = mode;
}

void BBKPhaseCorrectorAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                                     juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const auto numSamples = buffer.getNumSamples();
    const auto numChannels = buffer.getNumChannels();

    for (auto channel = getTotalNumInputChannels(); channel < getTotalNumOutputChannels(); ++channel)
        buffer.clear (channel, 0, numSamples);

    // Defensive: minimumBuffer/linearBuffer/delayedDryBuffer are normally
    // sized once in prepareToPlay() for the samplesPerBlock the host
    // declares there, for efficiency. Some hosts do not honour that as a
    // hard upper bound (an occasional larger "catch-up" block, adaptive
    // buffering, etc.), and jassert() below compiles out entirely in the
    // Release configuration this plugin ships as - so without this check,
    // a single oversized block would silently read/write past the end of
    // a fixed-size buffer via getSubBlock() (real, silent heap corruption,
    // not merely a debug-only assertion). Grow them here on the rare
    // occasion it's needed, the same defensive-resize pattern already
    // used for channel count elsewhere in this codebase (see Black-19's
    // PluginProcessor::process()).
    if (numSamples > minimumBuffer.getNumSamples())
    {
        minimumBuffer.setSize (getTotalNumOutputChannels(), numSamples, false, false, true);
        linearBuffer.setSize (getTotalNumOutputChannels(), numSamples, false, false, true);
        delayedDryBuffer.setSize (getTotalNumOutputChannels(), numSamples, false, false, true);
    }

    jassert (numSamples <= minimumBuffer.getNumSamples());
    jassert (numSamples <= linearBuffer.getNumSamples());
    jassert (numSamples <= delayedDryBuffer.getNumSamples());

    juce::dsp::AudioBlock<float> ioBlock (buffer);
    const juce::dsp::AudioBlock<const float> inputBlock (ioBlock);

    auto minimumBlock = juce::dsp::AudioBlock<float> (minimumBuffer).getSubBlock (0, static_cast<std::size_t> (numSamples));
    auto linearBlock = juce::dsp::AudioBlock<float> (linearBuffer).getSubBlock (0, static_cast<std::size_t> (numSamples));
    auto dryBlock = juce::dsp::AudioBlock<float> (delayedDryBuffer).getSubBlock (0, static_cast<std::size_t> (numSamples));

    minimumBlock.copyFrom (inputBlock);
    linearBlock.copyFrom (inputBlock);
    processDryDelay (inputBlock, dryBlock);

    // Pad before correcting (see the comment above applySafetySoftClip):
    // a linear, frequency-independent gain reduction so the phase-realigned
    // peak headroom this correction naturally uses up doesn't have to come
    // out of the correction's own peak shape via nonlinear limiting.
    // Applied only to the two corrected paths - BYPASS is untouched, so it
    // stays at full reference level regardless of this parameter.
    const float headroomDb = parameters.getRawParameterValue ("headroom")->load();
    const float preAttenuationGain = juce::Decibels::decibelsToGain (headroomDb);
    minimumBlock.multiplyBy (preAttenuationGain);
    linearBlock.multiplyBy (preAttenuationGain);

    juce::dsp::ProcessContextReplacing<float> minimumContext (minimumBlock);
    juce::dsp::ProcessContextReplacing<float> linearContext (linearBlock);
    minimumConvolution.process (minimumContext);
    linearConvolution.process (linearContext);

    const auto mode = juce::jlimit (0, 2,
        static_cast<int> (std::lround (parameters.getRawParameterValue ("mode")->load())));

    if (mode != lastMode)
        updateModeTargets (mode);

    // blockPeakOver tracks, across this whole block, how far the CORRECTED
    // signal alone (before the dry/BYPASS contribution is added back) pushed
    // past the soft-clip knee - this feeds the auto-headroom ratchet below,
    // and deliberately excludes the dry path so a hot BYPASS-only signal
    // can never trigger it.
    float blockPeakOver = 0.0f;

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const auto wb = bypassMix.getNextValue();
        const auto wm = minimumMix.getNextValue();
        const auto wl = linearMix.getNextValue();

        for (int channel = 0; channel < numChannels; ++channel)
        {
            const auto c = static_cast<std::size_t> (channel);
            const float dry = dryBlock.getChannelPointer (c)[sample];
            const float corrected = wm * minimumBlock.getChannelPointer (c)[sample]
                                   + wl * linearBlock.getChannelPointer (c)[sample];

            const float absCorrected = std::abs (corrected);
            if (absCorrected > softClipKneeStart)
                blockPeakOver = std::max (blockPeakOver, absCorrected - softClipKneeStart);

            // The soft-clip backstop applies ONLY to the corrected
            // contribution; the dry/BYPASS contribution is added back
            // afterwards, untouched. Previously the backstop was applied to
            // the full wb*dry + wm*min + wl*linear sum, which meant a
            // BYPASS signal anywhere near 0 dBFS - extremely common - was
            // being audibly soft-clipped even though BYPASS is supposed to
            // be a transparent pass-through. This was a genuine DSP bug,
            // not a design tradeoff.
            buffer.setSample (channel, sample, wb * dry + applySafetySoftClip (corrected));
        }
    }

    // Auto-headroom: one-way ratchet only (see autoAdjustCooldownSamples/
    // clipEnvelope in PluginProcessor.h for state, and the tuning constants
    // above). Runs once per block, not per sample - this is a slow,
    // "settle over several songs" adjustment, not a fast limiter.
    const bool autoHeadroomEnabled = parameters.getRawParameterValue ("autoHeadroom")->load() > 0.5f;
    if (autoHeadroomEnabled && currentSampleRate.load() > 0.0)
    {
        const float releaseThisBlock = autoHeadroomReleasePerSecond
                                      * static_cast<float> (numSamples)
                                      / static_cast<float> (currentSampleRate.load());
        clipEnvelope = std::max (blockPeakOver, clipEnvelope - releaseThisBlock);

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

juce::AudioProcessorEditor* BBKPhaseCorrectorAudioProcessor::createEditor()
{
    return new BBKPhaseCorrectorAudioProcessorEditor (*this);
}

void BBKPhaseCorrectorAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (const auto xml = parameters.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void BBKPhaseCorrectorAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (const auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (parameters.state.getType()))
            parameters.replaceState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BBKPhaseCorrectorAudioProcessor();
}
