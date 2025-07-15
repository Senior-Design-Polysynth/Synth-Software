#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc1, osc2;
AdcChannelConfig adcConfig[6];  // 6 controls

float volume1 = 0.f, volume2 = 0.f;
float pitch1 = 0.f, pitch2 = 0.f;
float pulseW1 = 0.f, pulseW2 = 0.f;  // Separate PWM for each oscillator
int currentWaveform1 = 0, currentWaveform2 = 0;  // Separate waveform states
bool lastButtonState1 = false, lastButtonState2 = false;  // Separate button states

void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    // Read all potentiometers
    volume1 = hw.adc.GetFloat(0);  // OSC1 volume
    pitch1 = hw.adc.GetFloat(1);   // OSC1 pitch
    pulseW1 = hw.adc.GetFloat(2);  // OSC1 pulse width
    volume2 = hw.adc.GetFloat(3);  // OSC2 volume
    pitch2 = hw.adc.GetFloat(4);   // OSC2 pitch
    pulseW2 = hw.adc.GetFloat(5);  // OSC2 pulse width

    // Configure oscillator 1
    osc1.SetFreq(50.f + (pitch1 * 1950.f));
    osc1.SetAmp(volume1);
    osc1.SetPw(pulseW1);

    // Configure oscillator 2
    osc2.SetFreq(50.f + (pitch2 * 1950.f));
    osc2.SetAmp(volume2);
    osc2.SetPw(pulseW2);

    for(size_t i = 0; i < size; i++)
    {
        float sig1 = osc1.Process();
        float sig2 = osc2.Process();
        out[0][i] = sig1 + sig2;
        out[1][i] = sig1 + sig2;
    }
}

void UpdateWaveform1()
{
    currentWaveform1 = (currentWaveform1 + 1) % 3;
    switch(currentWaveform1)
    {
        case 0: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

void UpdateWaveform2()
{
    currentWaveform2 = (currentWaveform2 + 1) % 3;
    switch(currentWaveform2)
    {
        case 0: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    // Initialize oscillators
    osc1.Init(hw.AudioSampleRate());
    osc2.Init(hw.AudioSampleRate());
    osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
    osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);

    // Initialize buttons
    GPIO button1, button2;
    button1.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC1 waveform
    button2.Init(D13, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC2 waveform

    // Configure ADC
    adcConfig[0].InitSingle(A0);  // OSC1 Volume
    adcConfig[1].InitSingle(A1);  // OSC1 Pitch
    adcConfig[2].InitSingle(A2);  // OSC1 PWM
    adcConfig[3].InitSingle(A3);  // OSC2 Volume
    adcConfig[4].InitSingle(A4);  // OSC2 Pitch
    adcConfig[5].InitSingle(A5);  // OSC2 PWM
    hw.adc.Init(adcConfig, 6);
    hw.adc.Start();

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Handle OSC1 button (D14)
        bool currentButtonState1 = !button1.Read();  // Invert for active-low
        if(currentButtonState1 && !lastButtonState1) {
            UpdateWaveform1();
        }
        lastButtonState1 = currentButtonState1;
        
        // Handle OSC2 button (D13)
        bool currentButtonState2 = !button2.Read();  // Invert for active-low
        if(currentButtonState2 && !lastButtonState2) {
            UpdateWaveform2();
        }
        lastButtonState2 = currentButtonState2;
        
        System::Delay(10);  // Debounce delay
    }
}