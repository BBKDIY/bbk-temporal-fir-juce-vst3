cmake_minimum_required(VERSION 3.22)

project(BBKPhaseCorrector VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BBKPC_BUILD_SMOKE_TEST_ONLY "Build only the framework-independent phase-designer smoke test (no JUCE)" OFF)

add_executable(BBKPhaseCorrector_SmokeTest
    tools/designer_smoke_test.cpp
    Source/PhaseFilterDesigner.cpp
    Source/PhaseFilterDesigner.h
    Source/PhaseModel.h
)
target_include_directories(BBKPhaseCorrector_SmokeTest PRIVATE Source)
target_compile_features(BBKPhaseCorrector_SmokeTest PRIVATE cxx_std_17)

if(BBKPC_BUILD_SMOKE_TEST_ONLY)
    return()
endif()

include(FetchContent)

FetchContent_Declare(
    JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG 9.0.0
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(JUCE)

juce_add_plugin(BBKPhaseCorrector
    COMPANY_NAME "BBK Audio"
    BUNDLE_ID "com.bbkaudio.phasecorrector"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD FALSE
    PLUGIN_MANUFACTURER_CODE BbkA
    PLUGIN_CODE PhC1
    FORMATS VST3
    PRODUCT_NAME "BBK Phase Corrector"
    DESCRIPTION "Phase-only loudspeaker/system correction derived from REW measurement"
)

juce_generate_juce_header(BBKPhaseCorrector)

target_sources(BBKPhaseCorrector
    PRIVATE
        Source/PluginProcessor.cpp
        Source/PluginProcessor.h
        Source/PluginEditor.cpp
        Source/PluginEditor.h
        Source/PhaseFilterDesigner.cpp
        Source/PhaseFilterDesigner.h
        Source/PhaseModel.h
)

target_compile_definitions(BBKPhaseCorrector
    PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(BBKPhaseCorrector
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)
