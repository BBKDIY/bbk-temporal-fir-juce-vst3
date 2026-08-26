cmake_minimum_required(VERSION 3.22)

project(BBKTemporalFIR VERSION 1.0.0 LANGUAGES C CXX)

option(BBK_BUILD_DSP_TEST_ONLY "Build only the framework-independent DSP verification executable" OFF)

add_executable(BBKTemporalFIR_DSPTest
    Tests/DSPTest.cpp
    Source/TemporalFIRBank.h
)
target_compile_features(BBKTemporalFIR_DSPTest PRIVATE cxx_std_17)

add_executable(BBKDetachedPole_DSPTest
    Tests/DSPTestDetachedPole.cpp
    SourceDetachedPole/DetachedPoleFilter.h
    SourceDetachedPole/ParametricFIR.h
)
target_compile_features(BBKDetachedPole_DSPTest PRIVATE cxx_std_17)

if(BBK_BUILD_DSP_TEST_ONLY)
    return()
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/JUCE/CMakeLists.txt")
    add_subdirectory(JUCE)
else()
    include(FetchContent)
    FetchContent_Declare(
        JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG 8.0.15
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(JUCE)
endif()

juce_add_plugin(BBKTemporalFIR
    VERSION 1.0.0
    COMPANY_NAME "BBK"
    BUNDLE_ID "com.bbk.temporalfir"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD FALSE
    PLUGIN_MANUFACTURER_CODE Bbk1
    PLUGIN_CODE Tfir
    FORMATS VST3
    PRODUCT_NAME "BBK Temporal FIR"
    NEEDS_WEB_BROWSER FALSE
    NEEDS_CURL FALSE
)

target_sources(BBKTemporalFIR PRIVATE
    Source/TemporalFIRBank.h
    Source/PluginProcessor.h
    Source/PluginProcessor.cpp
    Source/PluginEditor.h
    Source/PluginEditor.cpp
)

target_compile_features(BBKTemporalFIR PRIVATE cxx_std_17)

target_compile_definitions(BBKTemporalFIR PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(BBKTemporalFIR
    PRIVATE
        juce::juce_audio_utils
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)

juce_add_plugin(BBKDetachedPole
    VERSION 1.0.0
    COMPANY_NAME "BBK"
    BUNDLE_ID "com.bbk.detachedpole"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD FALSE
    PLUGIN_MANUFACTURER_CODE Bbk1
    PLUGIN_CODE Dpol
    FORMATS VST3
    PRODUCT_NAME "BBK Parametric FIR"
    NEEDS_WEB_BROWSER FALSE
    NEEDS_CURL FALSE
)

target_sources(BBKDetachedPole PRIVATE
    SourceDetachedPole/DetachedPoleFilter.h
    SourceDetachedPole/ParametricFIR.h
    SourceDetachedPole/PluginProcessor.h
    SourceDetachedPole/PluginProcessor.cpp
    SourceDetachedPole/PluginEditor.h
    SourceDetachedPole/PluginEditor.cpp
)

target_compile_features(BBKDetachedPole PRIVATE cxx_std_17)

target_compile_definitions(BBKDetachedPole PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
)

target_link_libraries(BBKDetachedPole
    PRIVATE
        juce::juce_audio_utils
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)
