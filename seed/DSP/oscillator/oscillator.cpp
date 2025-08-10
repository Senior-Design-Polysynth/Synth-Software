#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"


using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
AdcChannelConfig adcConfig[7];

float volume1 = 0.f, volume2 = 0.f;
float pulseW1 = 0.f, pulseW2 = 0.f;
float detune = 0.f;

constexpr int kNumVoices = 2;
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];
constexpr int kNumKeys = 6;

struct Voice {
    bool active;
    int note_id;   // button index 0-5 or MIDI note number
    bool is_midi;  // true = midi, false = button
    uint32_t timestamp; // add this line
};

Voice voices[kNumVoices];
GPIO keybutton[kNumKeys];
bool prev_state[kNumKeys] = {0};
uint32_t note_counter = 0;

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
    if(v == -1) 
    { 
       uint32_t oldest = voices[0].timestamp;
        v = 0;
        for(int i=1;i<kNumVoices;++i)
        {
            if(voices[i].timestamp < oldest)
            {
                oldest = voices[i].timestamp;
                v = i;
            }
        }
    }
    voices[v].active = true;
    voices[v].note_id = note_id;
    voices[v].is_midi = is_midi;
    voices[v].timestamp = ++note_counter;
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
                    // If already assigned, skip to next button
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

void VoiceNoteOff(int note_id, bool is_midi)
{
    for(int i=0;i<kNumVoices;++i)
    {
        // Check if the voice is active and matches the note_id and is_midi flag}
        if(voices[i].active && voices[i].note_id==note_id && voices[i].is_midi==is_midi)
        {
            voices[i].active = false;
        }
    }
    // Reassign held notes after turning off a voice
    ReassignHeldNotes();
}

// MIDI event handling
void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
        VoiceNoteOn(m.data[0], true);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
        VoiceNoteOff(m.data[0], true);
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
    detune  = 1.0f - hw.adc.GetFloat(5);  // OSC2 detune

    //Calculate detune in cents (-50 to +50 cents)
    float detuneCents = (detune - 0.5f) * 100.0f;
    float detuneFactor = powf(2.0f, detuneCents / 1200.0f);

    // Process each voice
    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active)
        {
            float freq = KeyNoteFreq(voices[v].note_id);
            osc1[v].SetFreq(freq);
            osc1[v].SetAmp(volume1);
            osc1[v].SetPw(pulseW1);

            osc2[v].SetFreq(freq * detuneFactor);
            osc2[v].SetAmp(volume2);
            osc2[v].SetPw(pulseW2);
        }
        else
        {
            osc1[v].SetAmp(0.0f);
            osc2[v].SetAmp(0.0f);
        }
    }

    // Mix the outputs of all voices
    for(size_t i = 0; i < size; i++)
    {
        float mix = 0.0f;
        for(int v = 0; v < kNumVoices; ++v)
        {
            mix += osc1[v].Process();
            mix += osc2[v].Process();
        }
        out[0][i] = mix * 0.5f; // scale to avoid clipping
        out[1][i] = mix * 0.5f;
    }
    
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    for(int v = 0; v < kNumVoices; ++v) 
    {
    osc1[v].Init(hw.AudioSampleRate());
    osc2[v].Init(hw.AudioSampleRate());
    osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
    osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
    }

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