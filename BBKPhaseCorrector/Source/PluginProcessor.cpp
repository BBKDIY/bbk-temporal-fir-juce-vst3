#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <algorithm>
#include <cmath>

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

    const auto minimumDesign = bbk::PhaseFilterDesigner::design (
        sampleRate, bbk::PhaseFilterDesigner::Target::minimumPhase);
    const auto linearDesign = bbk::PhaseFilterDesigner::design (
        sampleRate, bbk::PhaseFilterDesigner::Target::linearPhase);

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

void BBKPhaseCorrectorAudioProcessor::processDryDelay (const juce::AudioBlock<const float>& input,
                                                        juce::AudioBlock<float>& output) noexcept
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

    juce::dsp::ProcessContextReplacing<float> minimumContext (minimumBlock);
    juce::dsp::ProcessContextReplacing<float> linearContext (linearBlock);
    minimumConvolution.process (minimumContext);
    linearConvolution.process (linearContext);

    const auto mode = juce::jlimit (0, 2,
        static_cast<int> (std::lround (parameters.getRawParameterValue ("mode")->load())));

    if (mode != lastMode)
        updateModeTargets (mode);

    for (int sample = 0; sample < numSamples; ++sample)
    {
        const auto wb = bypassMix.getNextValue();
        const auto wm = minimumMix.getNextValue();
        const auto wl = linearMix.getNextValue();

        for (int channel = 0; channel < numChannels; ++channel)
        {
            const auto c = static_cast<std::size_t> (channel);
            buffer.setSample (channel, sample,
                              wb * dryBlock.getChannelPointer (c)[sample]
                            + wm * minimumBlock.getChannelPointer (c)[sample]
                            + wl * linearBlock.getChannelPointer (c)[sample]);
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
