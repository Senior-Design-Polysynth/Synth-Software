#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc1, osc2;
AdcChannelConfig adcConfig[7];

constexpr int kNumVoices = 2;
constexpr int kNumKeys = 6;

struct Voice {
    bool active;
    int note_id;   // button index 0-5 or MIDI note number
    bool is_midi;  // true = midi, false = button
};

Voice voices[kNumVoices];
GPIO keybutton[kNumKeys];
bool prev_state[kNumKeys] = {0};
int lastVoiceUsed = 0;

MidiUartHandler midi;

// Map local button "notes" to frequencies
float KeyNoteFreq(int note_id)
{
    static const float midiNotes[6] = {60, 62, 64, 65, 67, 69}; // C4, D4, E4, F4, G4, A4
    if(note_id < kNumKeys)
        return mtof(midiNotes[note_id]);
    else
        return mtof(note_id); // MIDI note ID
}

// Voice allocation/stealing, used for both button and MIDI
void VoiceNoteOn(int note_id, bool is_midi)
{
    int v = -1;
    // find free voice
    for(int i=0;i<kNumVoices;++i)
        if(!voices[i].active) { v=i; break; }
    // if none, steal
    if(v == -1) { v=lastVoiceUsed; lastVoiceUsed=(lastVoiceUsed+1)%kNumVoices; }
    voices[v].active = true;
    voices[v].note_id = note_id;
    voices[v].is_midi = is_midi;
}

void VoiceNoteOff(int note_id, bool is_midi)
{
    for(int i=0;i<kNumVoices;++i)
        if(voices[i].active && voices[i].note_id==note_id && voices[i].is_midi==is_midi)
            voices[i].active = false;
}

// Audio callback
void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    float pulseW1, pulseW2;
    pulseW1 = hw.adc.GetFloat(2);
    pulseW2 = 1.0f - hw.adc.GetFloat(5);

    // Voice management (osc1 = voices[0], osc2 = voices[1])
    if(voices[0].active)
    {
        osc1.SetFreq(KeyNoteFreq(voices[0].note_id));
        osc1.SetAmp(1.0f);
    }
    else
        osc1.SetAmp(0.0f);

    if(voices[1].active)
    {
        osc2.SetFreq(KeyNoteFreq(voices[1].note_id));
        osc2.SetAmp(1.0f);
    }
    else
        osc2.SetAmp(0.0f);

    osc1.SetPw(pulseW1);
    osc2.SetPw(pulseW2);

    for(size_t i = 0; i < size; i++)
    {
        float sig1 = osc1.Process();
        float sig2 = osc2.Process();
        out[0][i] = sig1 + sig2;
        out[1][i] = sig1 + sig2;
    }
}

// MIDI event handling
void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
        VoiceNoteOn(m.data[0], true);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
        VoiceNoteOff(m.data[0], true);
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    osc1.Init(hw.AudioSampleRate());
    osc2.Init(hw.AudioSampleRate());
    osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
    osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);

    daisy::Pin button_pins[kNumKeys] = {D9, D10, D11, D12, D13, D14};
    for(int i=0;i<kNumKeys;++i)
        keybutton[i].Init(button_pins[i], GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    adcConfig[0].InitSingle(A0);  // OSC1 Volume
    adcConfig[1].InitSingle(A1);  // OSC1 Pitch
    adcConfig[2].InitSingle(A2);  // OSC1 PWM
    adcConfig[3].InitSingle(A3);  // OSC2 Volume
    adcConfig[4].InitSingle(A4);  // OSC2 Pitch
    adcConfig[5].InitSingle(A5);  // OSC2 PWM
    adcConfig[6].InitSingle(A6);  // Key/Root control
    hw.adc.Init(adcConfig, 7);
    hw.adc.Start();

    // MIDI
    MidiUartHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
    midi_cfg.transport_config.rx = D30;
    midi_cfg.transport_config.tx = D29;
    midi.Init(midi_cfg);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        // Button keys
        for(int i=0;i<kNumKeys;++i)
        {
            bool pressed = !keybutton[i].Read();
            if(pressed && !prev_state[i])
                VoiceNoteOn(i, false);
            else if(!pressed && prev_state[i])
                VoiceNoteOff(i, false);
            prev_state[i] = pressed;
        }
        // MIDI
        midi.Listen();
        while(midi.HasEvents())
            HandleMidiMessage(midi.PopEvent());

        System::Delay(10);
    }
}