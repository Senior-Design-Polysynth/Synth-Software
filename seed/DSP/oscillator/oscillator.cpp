#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"


using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc1, osc2;
AdcChannelConfig adcConfig[7];

float volume1 = 0.f, volume2 = 0.f;
float pulseW1 = 0.f, pulseW2 = 0.f;
float detune = 0.f;

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
    {
        // Check if the voice is active and matches the note_id and is_midi flag}
        if(voices[i].active && voices[i].note_id==note_id && voices[i].is_midi==is_midi)
        {
            voices[i].active = false;
        }
        // If a voice is stolen, we need to reassign held notes
            ReassignHeldNotes();
    
    }
}


// Call this after VoiceNoteOff or after a voice is stolen
void ReassignHeldNotes()
{
    for(int i = 0; i < kNumKeys; ++i)
    {
        if(prev_state[i]) // button is held
        {
            // Check if this note is already assigned to a voice
            bool assigned = false;
            for(int v = 0; v < kNumVoices; ++v)
            {
                if(voices[v].active && voices[v].note_id == i && !voices[v].is_midi)
                {
                    assigned = true;
                    break;
                }
            }
            // If not assigned, assign it to a free voice
            if(!assigned)
                VoiceNoteOn(i, false);
        }
    }
}




// Audio callback
void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    // Read all potentiometers
    volume1 = hw.adc.GetFloat(0);  // OSC1 volume
    pulseW1 = hw.adc.GetFloat(1);  // OSC1 pulse width

    volume2 = 1.0f - hw.adc.GetFloat(3);  // OSC2 volume
    pulseW2 = 1.0f - hw.adc.GetFloat(4);  // OSC2 pulse width
    detune = 1.0f - hw.adc.GetFloat(5);   // OSC2 detune

    //Calculate detune in cents (-50 to +50 cents)
    float detuneCents = (detune - 0.5f) * 100.0f;
    float detuneFactor = powf(2.0f, detuneCents / 1200.0f);

    // Voice management (osc1 = voices[0], osc2 = voices[1])
    if(voices[0].active)
    {
        float freq1 = KeyNoteFreq(voices[0].note_id);
        osc1.SetFreq(freq1);
        osc1.SetAmp(volume1);
        //osc1.SetPw(pulseW1);
    }
    else
    {
        osc1.SetAmp(0.0f);
    }

    if(voices[1].active)
    {
        float freq2 = KeyNoteFreq(voices[1].note_id);
        osc2.SetFreq(freq2 * detuneFactor);
        osc2.SetAmp(volume2);
        //osc2.SetPw(pulseW2);
    }
    else
    {
        osc2.SetAmp(0.0f);
    } 


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
    osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
    osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);

    daisy::Pin button_pins[kNumKeys] = {D9, D10, D11, D12, D13, D14};
    for(int i=0;i<kNumKeys;++i)
        keybutton[i].Init(button_pins[i], GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    adcConfig[0].InitSingle(A0);    // OSC1 Volume
    adcConfig[1].InitSingle(A1);    // OSC1 Pulse Width
    //adcConfig[2].InitSingle(A2);  // OSC1 Detune (not used)
    adcConfig[3].InitSingle(A3);    // OSC2 Volume
    adcConfig[4].InitSingle(A4);    // OSC2 Pulse Width
    adcConfig[5].InitSingle(A5);    // OSC2 Detune
    //adcConfig[6].InitSingle(A6);  // unused
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