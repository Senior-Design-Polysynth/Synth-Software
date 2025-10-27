// File: src/midi_only_poly_synth.cpp
// Purpose: 4-voice, 2-osc-per-voice synth driven **only by external MIDI**.
// - Voice allocation: up to 4 simultaneous MIDI notes. On additional NoteOn, steal the OLDEST.
// - Restitution: if a stolen note is still held and the stealing note releases, the voice is reassigned to the oldest waiting held note.
// - Outputs to BOTH internal codec (SAI1: out[0], out[1]) **and** external PCM3060 on SAI2 (out[2], out[3]).
// - Two hardware buttons (D14, D13) are kept ONLY for changing waveforms of osc1/osc2 (not for playing notes).
// - FIXED: Corrected ADG706 enable pin to active-HIGH

#include "daisysp.h"
#include "daisy_seed.h" 
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ===== Hardware =====
DaisySeed hw;
static constexpr int kNumVoices = 4; // change freely (multiple of 1)

// Two buttons to cycle waveforms (NOT key notes)
int  currentWaveform1 = 0, currentWaveform2 = 0;
bool lastButtonState1 = false, lastButtonState2 = false;

// ===== ADC knobs (5 contiguous entries) =====
// order: A0(OSC1 Vol), A1(OSC1 PW), A3(OSC2 Vol), A4(OSC2 PW), A5(OSC2 Detune)
static constexpr int kNumAdc = 6;
AdcChannelConfig adc_cfg[kNumAdc];


// ===== Params read each audio block =====
float volume1 = 0.f, volume2 = 0.f;
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune1 = 0.5f; // 0..1 => -200..+200 cents
float detune2 = 0.5f; // 0..1 => -200..+200 cents

// ===== Synthesis =====
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];

// ===== MIDI =====
MidiUartHandler midi;

// ===== Voice/Allocation State (MIDI-only) =====
struct Voice {
    bool     active = false;
    int      note   = -1;    // MIDI note number 0..127
};

Voice voices[kNumVoices];

// Per-MIDI-note state
static constexpr int kNumMidiNotes = 128;
bool     midi_held[kNumMidiNotes] = {false};
int      midi_voice[kNumMidiNotes];        // -1 if none, else voice index
uint32_t midi_hold_ts[kNumMidiNotes];      // press order

uint32_t global_press_counter = 0; // monotonic press counter

static inline int  ActiveVoiceCount() { int c=0; for(int v=0; v<kNumVoices; ++v) c += voices[v].active?1:0; return c; }

static inline int FindFreeVoice()
{
  for(int v=0; v<kNumVoices; ++v)
    if(!voices[v].active) return v; 
  return -1;
}
// Among ACTIVE voices, pick index whose owner has the smallest (oldest) hold timestamp
static int FindOldestActiveVoice()
{
    int oldest_vi = -1;
    uint32_t oldest_ts = 0;
    for(int v=0; v<kNumVoices; ++v)
    {
        if(!voices[v].active) continue;
        const int n = voices[v].note;
        const uint32_t ts = midi_hold_ts[n];
        if(oldest_vi < 0 || ts < oldest_ts) { oldest_ts = ts; oldest_vi = v; }
    }
    return oldest_vi;
}

// Among HELD but UNASSIGNED MIDI notes, return the one with oldest ts
static int FindOldestWaitingMidiNote()
{
    int best = -1; 
    uint32_t oldest_ts = 0;
    bool have = false;
    for(int n=0; n<kNumMidiNotes; ++n)
    {
        if(midi_held[n] && midi_voice[n] < 0)
        {
            if(!have || midi_hold_ts[n] < oldest_ts) { oldest_ts = midi_hold_ts[n]; best = n; have = true; }
        }
    }
    return best; // -1 if none
}

static void AssignVoiceToMidi(int voice_idx, int note)
{
    voices[voice_idx].active = true;
    voices[voice_idx].note   = note;
    midi_voice[note] = voice_idx;
}

static void ReleaseVoice(int voice_idx)
{
    if(!voices[voice_idx].active) return;
    const int note = voices[voice_idx].note;
    if(note >= 0 && midi_voice[note] == voice_idx)
        midi_voice[note] = -1;
    voices[voice_idx].active = false;
    voices[voice_idx].note   = -1;
}

static void OnMidiNoteOn(int note)
{
    if(note < 0 || note >= kNumMidiNotes) return;
    midi_held[note] = true;
    midi_hold_ts[note] = ++global_press_counter;

    int free_v = FindFreeVoice();
    if(free_v >= 0) { AssignVoiceToMidi(free_v, note); return; }

    // steal from oldest active
    int steal_vi = FindOldestActiveVoice();
    if(steal_vi >= 0)
    {
        const int victim_note = voices[steal_vi].note;
        midi_voice[victim_note] = -1; // victim remains held but unassigned
        AssignVoiceToMidi(steal_vi, note);
    }
}

static void OnMidiNoteOff(int note)
{
    if(note < 0 || note >= kNumMidiNotes) return;
    midi_held[note] = false;

    int owned_v = midi_voice[note];
    if(owned_v >= 0)
        ReleaseVoice(owned_v);

    // Restitution: hand any free voices to oldest waiting MIDI notes
    while(true)
    {
        int wait_note = FindOldestWaitingMidiNote();
        int v = FindFreeVoice();
        if(wait_note < 0 || v < 0) break;
        AssignVoiceToMidi(v, wait_note);
    }
}

// ===== MIDI event handling =====
static void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
      OnMidiNoteOn(m.data[0]);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
      OnMidiNoteOff(m.data[0]);
}

// ===== Audio Callback =====
static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    // pots (match contiguous ADC order)
    volume1  = hw.adc.GetFloat(0);
    pulseW1  = hw.adc.GetFloat(1);
    detune1  = hw.adc.GetFloat(2);
    volume2  = hw.adc.GetFloat(3);
    pulseW2  = hw.adc.GetFloat(4);
    detune2  = hw.adc.GetFloat(5);

    const float cents1 = (detune1 - 0.5f) * 400.0f;
    const float cents2 = (detune2 - 0.5f) * 400.0f;
    const float detuneFactor1 = powf(2.0f, cents1 / 1200.0f);
    const float detuneFactor2 = powf(2.0f, cents2 / 1200.0f);

    // Normalize by ACTIVE voices (2 oscs per voice)
    //const int active_v = ActiveVoiceCount();
    //const float mix_scale = active_v > 0 ? 1.0f / (2.0f * active_v) : 0.0f;
    //const float mix_scale = active_v > 0 ? 1.0f  : 0.0f;
    const float mix_scale = 1.f / (2.f * kNumVoices);

    // Per-voice parameter update
    for(int v=0; v<kNumVoices; ++v)
    {
        if(voices[v].active)
        {
            const float f = mtof(static_cast<float>(voices[v].note));
            osc1[v].SetFreq(f * detuneFactor1);
            osc1[v].SetAmp(volume1);
            osc1[v].SetPw(pulseW1);

            osc2[v].SetFreq(f * detuneFactor2);
            osc2[v].SetAmp(volume2);
            osc2[v].SetPw(pulseW2);
        }
        else
        {
            osc1[v].SetAmp(0.f);
            osc2[v].SetAmp(0.f);
        }
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
}

//======== Waveform change functions ========
void UpdateWaveform1()
{
    currentWaveform1 = (currentWaveform1 + 1) % 3;
    for(int v = 0; v < kNumVoices; ++v)
    {
        switch(currentWaveform1)
        {
            case 0: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
            case 1: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);    break;
            case 2: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);    break;
        }
    }
}

void UpdateWaveform2()
{
    currentWaveform2 = (currentWaveform2 + 1) % 3;
    for(int v = 0; v < kNumVoices; ++v)
    {
        switch(currentWaveform2)
        {
            case 0: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
            case 1: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);    break;
            case 2: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);    break;
        }
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(48); // 1ms @ 48kHz; multiple of 4

    // Buttons for waveform switching only
    GPIO button1, button2;
    button1.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC1 waveform
    button2.Init(D13, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC2 waveform

    // MIDI UART (DIN)
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
    adc_cfg[2].InitSingle(A2); // OSC1 Detune
    adc_cfg[3].InitSingle(A3); // OSC2 Volume
    adc_cfg[4].InitSingle(A4); // OSC2 Pulse Width
    adc_cfg[5].InitSingle(A5); // OSC2 Detune
    hw.adc.Init(adc_cfg, kNumAdc);
    hw.adc.Start();

    // Oscillators
    for(int v=0; v<kNumVoices; ++v)
    {
        osc1[v].Init(hw.AudioSampleRate());
        osc2[v].Init(hw.AudioSampleRate());
        osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
        osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
        osc1[v].SetAmp(0.f);
        osc2[v].SetAmp(0.f);
    }

    // Clear MIDI note ownership maps
    for(int n=0; n<kNumMidiNotes; ++n) { midi_voice[n] = -1; midi_hold_ts[n] = 0; }

    // ===== SAI2 (external PCM3060) config =====
    SaiHandle         sai2;
    SaiHandle::Config sc;
    sc.periph    = SaiHandle::Config::Peripheral::SAI_2;
    sc.sr        = SaiHandle::Config::SampleRate::SAI_48KHZ;
    sc.bit_depth = SaiHandle::Config::BitDepth::SAI_24BIT;
    // Set protocol to match your PCM3060 straps:
    //sc.protocol  = SaiHandle::Config::Protocol::LEFT_JUSTIFIED; // or ::I2S if strapped so

    sc.a_sync    = SaiHandle::Config::Sync::SLAVE;   // SD_A (codec->MCU)
    sc.b_sync    = SaiHandle::Config::Sync::MASTER;  // clocks on *_B pins
    sc.a_dir     = SaiHandle::Config::Direction::RECEIVE;   // SD_A = D26
    sc.b_dir     = SaiHandle::Config::Direction::TRANSMIT;  // SD_B = D25

    sc.pin_config.mclk = D24; // SAI2_MCLK_B
    sc.pin_config.sck  = D28; // SAI2_SCK_B
    sc.pin_config.fs   = D27; // SAI2_FS_B
    sc.pin_config.sa   = D26; // SAI2_SD_A (codec->MCU)
    sc.pin_config.sb   = D25; // SAI2_SD_B (MCU->codec)

    sai2.Init(sc);

    // Dual-SAI (internal + external)
    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 48; // multiple of 4
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 0.5f;

    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    hw.StartAudio(AudioCallback);

    // Main loop
    while(true)
    {
        // Waveform buttons
        bool cur1 = !button1.Read();
        if(cur1 && !lastButtonState1)
          UpdateWaveform1();
        lastButtonState1 = cur1;
      
        bool cur2 = !button2.Read();
        if(cur2 && !lastButtonState2)
          UpdateWaveform2();
        lastButtonState2 = cur2;

        // MIDI processing
        midi.Listen();
        while(midi.HasEvents())
          HandleMidiMessage(midi.PopEvent());

        
        System::Delay(1);
    }
}
