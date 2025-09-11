#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisysp;
using namespace daisy;

static DaisySeed  hw;
static Oscillator osc;

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    float sig;
    // We have 4 channels total (2 internal, 2 external).
    // The buffer size is blocksize * 4. We need to step by 4.
    for(size_t i = 0; i < size; ++i)
    {
        sig = osc.Process();

        // Channels 0 and 1 are for the INTERNAL codec
        out[0][i] = sig; // Internal Left
        out[1][i] = sig; // Internal Right

        // Channels 2 and 3 are for the EXTERNAL PCM3060
        out[2][i] = sig; // External Left
        out[3][i] = sig; // External Right
    }
}

int main(void)
{
    // initialize seed hardware and oscillator daisysp module
    float sample_rate;
    hw.Configure();
    hw.Init();
    //hw.SetAudioBlockSize(4);
    sample_rate = hw.AudioSampleRate();
    osc.Init(sample_rate);

    // Set parameters for oscillator
    osc.SetWaveform(osc.WAVE_SIN);
    osc.SetFreq(440);
    osc.SetAmp(0.5);

    SaiHandle         sai2;
    SaiHandle::Config sc;
    sc.periph          = SaiHandle::Config::Peripheral::SAI_2;
    sc.sr              = SaiHandle::Config::SampleRate::SAI_48KHZ;
    sc.bit_depth       = SaiHandle::Config::BitDepth::SAI_24BIT;

    sc.a_sync          = SaiHandle::Config::Sync::SLAVE;
    sc.b_sync          = SaiHandle::Config::Sync::MASTER;
    sc.a_dir           = SaiHandle::Config::Direction::RECEIVE;
    sc.b_dir           = SaiHandle::Config::Direction::TRANSMIT;
    sc.pin_config.fs   = seed::D27;
    sc.pin_config.mclk = seed::D24;
    sc.pin_config.sck  = seed::D28;
    sc.pin_config.sb   = seed::D25;
    sc.pin_config.sa   = seed::D26;

    //Initialize the SAI new handle 
    sai2.Init(sc);

    //Reconfigure Audio for two codecs 
    //
    //  Default eurorack circuit has an extra 6dB headroom 
    //  so the 0.5 here makes it so that a -1 to 1 audio signal
    //  will correspond to a -5V to 5V (10Vpp) audio signal.
    //  Audio will clip at -2 to 2, and result 20Vpp output.
    //


    /*AudioHandle ext;
    AudioHandle::Config ac;
    ac.blocksize  = 48;
    ac.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    ext.Init(ac, sai2);
    ext.Start(AudioCallback);*/


    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 48;
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 0.5f;

    // Initialize for two SAIs, including the built-in SAI that is 
    //  configured during hw.Init()
    //
    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    // Finally start the audio
    hw.StartAudio(AudioCallback);


    while(1) {}
}




 for(size_t i = 0; i < size; ++i)
    {
        float mix[kNumVoices];

        for(int v = 0; v < kNumVoices; ++v)
        {
            mix[v] = osc1[v].Process() + osc2[v].Process(); 
            mix[v] *= mix_scale; // headroom scales with polyphony
        }
       
        // Channels 0 and 1 are for the INTERNAL codec
        out[0][i] = mix[0]; // Internal Left
        out[1][i] = mix[1]; // Internal Right

        // Channels 2 and 3 are for the EXTERNAL PCM3060
        out[2][i] = mix[2]; // External Left
        out[3][i] = mix[3]; // External Right
    }
