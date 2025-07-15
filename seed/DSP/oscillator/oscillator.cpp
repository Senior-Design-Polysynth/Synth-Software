#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc1, osc2, osc3, osc4;
AdcChannelConfig adcConfig[6];

// Parameters for both voices
struct VoiceParams {
    float vol1, vol2;
    float pitch1, pitch2;
    float pw1, pw2;
    int wave1, wave2;
} voice1, voice2;

int currentVoice = 0;
bool lastButtonState1 = false, lastButtonState2 = false, lastButtonStateVoice = false;

// Helper function to set waveform based on index
void SetOscWaveform(Oscillator& osc, int index) {
    switch(index) {
        case 0: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

// Convert pitch value to quantized frequency (chromatic scale)
float QuantizePitch(float pitch) {
    // Define chromatic scale range (MIDI notes 24-108 = C1-C8)
    const float minNote = 24.0f;
    const float maxNote = 108.0f;
    const float range = maxNote - minNote;
    
    // Calculate MIDI note number
    float midiNote = minNote + (pitch * range);
    
    // Quantize to nearest integer note
    midiNote = roundf(midiNote);
    
    // Convert MIDI note to frequency (A4 = 440Hz)
    return 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);

// To quantize to C major scale instead of chromatic
// int notesInOctave[] = {0, 2, 4, 5, 7, 9, 11}; // C major scale degrees
// int octave = (int)midiNote / 12;
// int noteInOctave = (int)midiNote % 12;

// // Find nearest scale note
// int closestNote = 0;
// int minDistance = 12;
// for(int i = 0; i < 7; i++) {
//     int distance = abs(noteInOctave - notesInOctave[i]);
//     if(distance < minDistance) {
//         minDistance = distance;
//         closestNote = notesInOctave[i];
//     }
// }
// midiNote = octave * 12 + closestNote;


}




void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        float mixed = osc1.Process() + osc2.Process() + osc3.Process() + osc4.Process();
        out[0][i] = mixed;
        out[1][i] = mixed;
    }
}

void UpdateWaveform1()
{
    if(currentVoice == 0) {
        voice1.wave1 = (voice1.wave1 + 1) % 3;
    } else {
        voice2.wave1 = (voice2.wave1 + 1) % 3;
    }
}

void UpdateWaveform2()
{
    if(currentVoice == 0) {
        voice1.wave2 = (voice1.wave2 + 1) % 3;
    } else {
        voice2.wave2 = (voice2.wave2 + 1) % 3;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    // Initialize all oscillators
    osc1.Init(hw.AudioSampleRate());
    osc2.Init(hw.AudioSampleRate());
    osc3.Init(hw.AudioSampleRate());
    osc4.Init(hw.AudioSampleRate());
    
    // Initialize voice parameters
    voice1 = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 2, 2};
    voice2 = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 2, 2};

    // Initialize buttons
    GPIO buttonWave1, buttonWave2, buttonVoice;
    buttonWave1.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    buttonWave2.Init(D13, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    buttonVoice.Init(D12, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    // Configure ADC
    adcConfig[0].InitSingle(A0);
    adcConfig[1].InitSingle(A1);
    adcConfig[2].InitSingle(A2);
    adcConfig[3].InitSingle(A3);
    adcConfig[4].InitSingle(A4);
    adcConfig[5].InitSingle(A5);
    hw.adc.Init(adcConfig, 6);
    hw.adc.Start();

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Update current voice parameters from pots
        if(currentVoice == 0) {
            voice1.vol1 = hw.adc.GetFloat(0);
            voice1.pitch1 = hw.adc.GetFloat(1);
            voice1.pw1 = hw.adc.GetFloat(2);
            voice1.vol2 = hw.adc.GetFloat(3);
            voice1.pitch2 = hw.adc.GetFloat(4);
            voice1.pw2 = hw.adc.GetFloat(5);
        } else {
            voice2.vol1 = hw.adc.GetFloat(0);
            voice2.pitch1 = hw.adc.GetFloat(1);
            voice2.pw1 = hw.adc.GetFloat(2);
            voice2.vol2 = hw.adc.GetFloat(3);
            voice2.pitch2 = hw.adc.GetFloat(4);
            voice2.pw2 = hw.adc.GetFloat(5);
        }

        // Voice 1 oscillators (with quantized pitch)
        osc1.SetFreq(QuantizePitch(voice1.pitch1));
        osc1.SetAmp(voice1.vol1);
        osc1.SetPw(voice1.pw1);
        SetOscWaveform(osc1, voice1.wave1);
        
        osc2.SetFreq(QuantizePitch(voice1.pitch2));
        osc2.SetAmp(voice1.vol2);
        osc2.SetPw(voice1.pw2);
        SetOscWaveform(osc2, voice1.wave2);
        
        // Voice 2 oscillators (with quantized pitch)
        osc3.SetFreq(QuantizePitch(voice2.pitch1));
        osc3.SetAmp(voice2.vol1);
        osc3.SetPw(voice2.pw1);
        SetOscWaveform(osc3, voice2.wave1);
        
        osc4.SetFreq(QuantizePitch(voice2.pitch2));
        osc4.SetAmp(voice2.vol2);
        osc4.SetPw(voice2.pw2);
        SetOscWaveform(osc4, voice2.wave2);

        // Handle waveform buttons
        bool currentButtonState1 = !buttonWave1.Read();
        if(currentButtonState1 && !lastButtonState1) UpdateWaveform1();
        lastButtonState1 = currentButtonState1;
        
        bool currentButtonState2 = !buttonWave2.Read();
        if(currentButtonState2 && !lastButtonState2) UpdateWaveform2();
        lastButtonState2 = currentButtonState2;
        
        // Handle voice select button
        bool currentButtonStateVoice = !buttonVoice.Read();
        if(currentButtonStateVoice && !lastButtonStateVoice) {
            currentVoice = 1 - currentVoice;
        }
        lastButtonStateVoice = currentButtonStateVoice;
        
        System::Delay(10);
    }
}