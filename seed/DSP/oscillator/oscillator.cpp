// File: src/button_synth_two_voices.cpp
// Purpose: Two-voice, two-osc-per-voice synth driven by hardware buttons **and** external MIDI over DIN.
// Behavior:
//  - Up to 2 simultaneous notes total (buttons + MIDI combined) => 2 voices, each with 2 oscillators.
//  - On 3rd (or more) press while 2 voices active: steal the OLDEST held note (fair voice stealing).
//  - If a stolen note remains held and the stealing note releases, the voice is returned to the stolen note (restitution).
//  - Buttons map to fixed notes; MIDI uses incoming note numbers.
// Why: Unified allocator across buttons & MIDI, minimal races, fixed ADC config, edge-driven logic.

#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ===== Hardware =====
DaisySeed hw;
static constexpr int kNumVoices = 2;
static constexpr int kNumKeys   = 6;               // hardware buttons
static const Pin kButtonPins[kNumKeys] = {D9, D10, D11, D12, D13, D14};
GPIO keybutton[kNumKeys];

// ===== ADC knobs (5 contiguous entries) =====
// order: A0(OSC1 Vol), A1(OSC1 PW), A3(OSC2 Vol), A4(OSC2 PW), A5(OSC2 Detune)
static constexpr int kNumAdc = 5;
AdcChannelConfig adc_cfg[kNumAdc];

// ===== Params read each audio block =====
float volume1 = 0.f, volume2 = 0.f;
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune  = 0.5f; // 0..1 => -50..+50 cents

// ===== Synthesis =====
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];

// ===== MIDI =====
MidiUartHandler midi;

// Buttons -> fixed MIDI notes
static inline float ButtonNoteFreq(int btn_id)
{
    static const uint8_t kMidiNotes[kNumKeys] = {60, 62, 64, 65, 67, 69}; // C4 D4 E4 F4 G4 A4
    return mtof(kMidiNotes[btn_id]);
}

// ===== Voice/Allocation State =====
struct Voice {
    bool     active = false;
    bool     is_midi = false; // owner type
    int      key_id = -1;     // button index 0..5 or MIDI note 0..127
};

Voice voices[kNumVoices];

// Per-button state
bool     btn_held[kNumKeys]   = {false};
int      btn_voice[kNumKeys];              // -1 if none, else voice index
uint32_t btn_hold_ts[kNumKeys];            // press order

// Per-MIDI-note state
static constexpr int kNumMidiNotes = 128;
bool     midi_held[kNumMidiNotes] = {false};
int      midi_voice[kNumMidiNotes];        // -1 if none
uint32_t midi_hold_ts[kNumMidiNotes];

uint32_t global_press_counter = 0; // monotonic; used for order only

// ---- Small helpers (keep logic readable) ----
static inline bool   GetHeld(bool is_midi, int id)               { return is_midi ? midi_held[id]      : btn_held[id]; }
static inline void   SetHeld(bool is_midi, int id, bool v)       { if(is_midi) midi_held[id]=v; else btn_held[id]=v; }
static inline int    GetOwnerVoice(bool is_midi, int id)         { return is_midi ? midi_voice[id]     : btn_voice[id]; }
static inline void   SetOwnerVoice(bool is_midi, int id, int vi) { if(is_midi) midi_voice[id]=vi; else btn_voice[id]=vi; }
static inline uint32_t GetTs(bool is_midi, int id)               { return is_midi ? midi_hold_ts[id]   : btn_hold_ts[id]; }
static inline void     SetTs(bool is_midi, int id, uint32_t ts)  { if(is_midi) midi_hold_ts[id]=ts; else btn_hold_ts[id]=ts; }
static inline float  KeyFreq(bool is_midi, int id)               { return is_midi ? mtof((float)id) : ButtonNoteFreq(id); }

struct KeyRef { bool is_midi=false; int id=-1; bool valid=false; };

static int FindFreeVoice()
{
    for(int v = 0; v < kNumVoices; ++v)
        if(!voices[v].active) return v;
    return -1;
}

// Among ACTIVE voices, pick index whose owner has the smallest hold timestamp (oldest)
static int FindOldestActiveVoice()
{
    int oldest_vi = -1;
    uint32_t oldest_ts = 0;
    for(int v = 0; v < kNumVoices; ++v)
    {
        if(!voices[v].active) continue;
        uint32_t ts = GetTs(voices[v].is_midi, voices[v].key_id);
        if(oldest_vi < 0 || ts < oldest_ts) { oldest_ts = ts; oldest_vi = v; }
    }
    return oldest_vi;
}

// Among HELD but UNASSIGNED keys (buttons and MIDI), return the one with the oldest ts
static KeyRef FindOldestWaitingKey()
{
    KeyRef best; uint32_t oldest_ts = 0;
    // buttons
    for(int b = 0; b < kNumKeys; ++b)
    {
        if(btn_held[b] && btn_voice[b] < 0)
        {
            uint32_t ts = btn_hold_ts[b];
            if(!best.valid || ts < oldest_ts) { oldest_ts = ts; best = {false, b, true}; }
        }
    }
    // MIDI notes
    for(int n = 0; n < kNumMidiNotes; ++n)
    {
        if(midi_held[n] && midi_voice[n] < 0)
        {
            uint32_t ts = midi_hold_ts[n];
            if(!best.valid || ts < oldest_ts) { oldest_ts = ts; best = {true, n, true}; }
        }
    }
    return best;
}

static void AssignVoiceToKey(int voice_idx, bool is_midi, int key_id)
{
    voices[voice_idx].active = true;
    voices[voice_idx].is_midi = is_midi;
    voices[voice_idx].key_id = key_id;
    SetOwnerVoice(is_midi, key_id, voice_idx);
}

static void ReleaseVoice(int voice_idx)
{
    if(!voices[voice_idx].active) return;
    const bool is_midi = voices[voice_idx].is_midi;
    const int  key_id  = voices[voice_idx].key_id;
    if(key_id >= 0)
    {
        if(GetOwnerVoice(is_midi, key_id) == voice_idx)
            SetOwnerVoice(is_midi, key_id, -1);
    }
    voices[voice_idx].active = false;
    voices[voice_idx].key_id = -1;
}

static void OnKeyPressed(bool is_midi, int id)
{
    // ignore out-of-range MIDI ids
    if(is_midi && (id < 0 || id >= kNumMidiNotes)) return;
    SetHeld(is_midi, id, true);
    SetTs(is_midi, id, ++global_press_counter);

    int free_v = FindFreeVoice();
    if(free_v >= 0)
    {
        AssignVoiceToKey(free_v, is_midi, id);
        return;
    }

    // Steal from oldest active
    int steal_vi = FindOldestActiveVoice();
    if(steal_vi >= 0)
    {
        const bool victim_midi = voices[steal_vi].is_midi;
        const int  victim_id   = voices[steal_vi].key_id;
        // victim remains held but unassigned
        SetOwnerVoice(victim_midi, victim_id, -1);
        AssignVoiceToKey(steal_vi, is_midi, id);
    }
}

static void OnKeyReleased(bool is_midi, int id)
{
    if(is_midi && (id < 0 || id >= kNumMidiNotes)) return;
    SetHeld(is_midi, id, false);

    int owned_v = GetOwnerVoice(is_midi, id);
    if(owned_v >= 0)
        ReleaseVoice(owned_v);

    // Give free voices (if any) to the oldest waiting keys (restitution first)
    while(true)
    {
        KeyRef wait = FindOldestWaitingKey();
        int v = FindFreeVoice();
        if(!wait.valid || v < 0)
            break;
        AssignVoiceToKey(v, wait.is_midi, wait.id);
    }
}

// ===== MIDI event handling =====
static void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
        OnKeyPressed(true, m.data[0]);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
        OnKeyReleased(true, m.data[0]);
}

// ===== Audio Callback =====
static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    // pots (match contiguous ADC order)
    volume1 = hw.adc.GetFloat(0);
    pulseW1 = hw.adc.GetFloat(1);
    volume2 = hw.adc.GetFloat(2);
    pulseW2 = hw.adc.GetFloat(3);
    detune  = hw.adc.GetFloat(4);

    const float cents = (detune - 0.5f) * 100.0f;
    const float detuneFactor = powf(2.0f, cents / 1200.0f);

    // Scale output based on max polyphony (2 oscs per voice)
    const float mix_scale = 1.0f / (2.0f * kNumVoices);

    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active)
        {
            const float f = KeyFreq(voices[v].is_midi, voices[v].key_id);
            osc1[v].SetFreq(f);
            osc1[v].SetAmp(volume1);
            osc1[v].SetPw(pulseW1);

            osc2[v].SetFreq(f * detuneFactor);
            osc2[v].SetAmp(volume2);
            osc2[v].SetPw(pulseW2);
        }
        else
        {
            // why: hard-zero inactive voices to avoid bleed
            osc1[v].SetAmp(0.f);
            osc2[v].SetAmp(0.f);
        }
    }

    for(size_t i = 0; i < size; ++i)
    {
        float mix = 0.f;
        for(int v = 0; v < kNumVoices; ++v)
        {
            mix += osc1[v].Process();
            mix += osc2[v].Process();
        }
        mix *= mix_scale; // headroom scales with polyphony
        out[0][i] = mix;
        out[1][i] = mix;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    // Buttons
    for(int i = 0; i < kNumKeys; ++i)
    {
        keybutton[i].Init(kButtonPins[i], GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
        btn_voice[i]   = -1;
        btn_hold_ts[i] = 0;
    }

    // Initialize MIDI UART (DIN)
    {
        MidiUartHandler::Config midi_cfg;
        midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
        midi_cfg.transport_config.rx = D30;
        midi_cfg.transport_config.tx = D29; // not required for input-only
        midi.Init(midi_cfg);
    }

    // ADC (contiguous entries only)
    adc_cfg[0].InitSingle(A0); // OSC1 Volume
    adc_cfg[1].InitSingle(A1); // OSC1 Pulse Width
    adc_cfg[2].InitSingle(A3); // OSC2 Volume
    adc_cfg[3].InitSingle(A4); // OSC2 Pulse Width
    adc_cfg[4].InitSingle(A5); // OSC2 Detune
    hw.adc.Init(adc_cfg, kNumAdc);
    hw.adc.Start();

    // Oscillators
    for(int v = 0; v < kNumVoices; ++v)
    {
        osc1[v].Init(hw.AudioSampleRate());
        osc2[v].Init(hw.AudioSampleRate());
        osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        osc1[v].SetAmp(0.f);
        osc2[v].SetAmp(0.f);
    }

    // Start audio
    hw.StartAudio(AudioCallback);

    // Poll buttons + process MIDI
    bool prev[kNumKeys] = {false};

    while(true)
    {
        // Buttons
        for(int b = 0; b < kNumKeys; ++b)
        {
            bool pressed = !keybutton[b].Read(); // active-low
            if(pressed && !prev[b])
                OnKeyPressed(false, b);
            else if(!pressed && prev[b])
                OnKeyReleased(false, b);
            prev[b] = pressed;
        }

        // MIDI
        midi.Listen();
        while(midi.HasEvents())
            HandleMidiMessage(midi.PopEvent());

        System::Delay(1);
    }
}
