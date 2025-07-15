#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc;
AdcChannelConfig adcConfig[3];

float volume = 0.f;
float pitch = 0.f;
float pulseW = 0.f;
int currentWaveform = 0;
bool lastButtonState = false;

void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    volume = hw.adc.GetFloat(0);
    pitch = hw.adc.GetFloat(1);
    pulseW = hw.adc.GetFloat(2);


    osc.SetFreq(50.f + (pitch * 1950.f));
    osc.SetAmp(volume);
    osc.SetPw(pulseW);

    for(size_t i = 0; i < size; i++)
    {
        float sig = osc.Process();
        out[0][i] = sig;
        out[1][i] = sig;
    }
}

void UpdateWaveform()
{
    currentWaveform = (currentWaveform + 1) % 3;
    switch(currentWaveform)
    {
        case 0: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    osc.Init(hw.AudioSampleRate());
    osc.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);

    GPIO button;
    button.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    adcConfig[0].InitSingle(A0);
    adcConfig[1].InitSingle(A1);
    adcConfig[2].InitSingle(A2);
    hw.adc.Init(adcConfig, 3);
    hw.adc.Start();

    hw.StartAudio(AudioCallback);

    while(1)
    {
        bool currentButtonState = !button.Read();
        if(currentButtonState && !lastButtonState)
        {
            UpdateWaveform();
        }
        lastButtonState = currentButtonState;
        System::Delay(10);
    }
}