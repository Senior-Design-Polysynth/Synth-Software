// File: src/oscillator.cpp
// Purpose: 4-voice synth with Analog Envelopes + PITCH BEND support.
//
// FIXED: Truncation error (restored missing main loop).
// FIXED: "Defined but not used" warnings.
// ADDED: Pitch Bend (+/- 2 Semitones).

#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ===== Hardware =====
DaisySeed hw;
static constexpr int kNumVoices = 4; 

UartHandler uart;

// ------------------- Constants for framing -------------------
static constexpr uint8_t HEADER1 = 0xAA;
static constexpr uint8_t HEADER2 = 0x55;
static constexpr int     MAX_FRAME = 128;

// ------------------- Receive buffer variables -------------------
uint8_t rxBuf[MAX_FRAME];
int     rxIndex = 0;
int     expectedLen = 0;
int     state = 0;
static uint16_t g_ctrl_buf[11];
static uint8_t  g_ctrl_parts = 0;

// ===== Params =====
float volume1 = 0.5f, volume2 = 0.5f; 
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune1 = 0.5f;
float detune2 = 0.5f;

// ===== Pitch Bend State =====
// 1.0 = no bend. >1.0 = sharp, <1.0 = flat.
float g_bend_mult = 1.0f; 

// ===== Synthesis =====
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];

// ===== MIDI =====
MidiUartHandler midi;

// ===== Voice/Allocation State =====
struct Voice {
    bool     active = false;
    bool     gate   = false;
    int      note   = -1;    
};

Voice voices[kNumVoices];

// Per-MIDI-note state
static constexpr int kNumMidiNotes = 128;
bool     midi_held[kNumMidiNotes] = {false};
int      midi_voice[kNumMidiNotes];        
uint32_t midi_hold_ts[kNumMidiNotes];      

uint32_t global_press_counter = 0;

// --- GATES D15-D18 ---
static const Pin kGatePins[4] = { D15, D16, D17, D18 };
static GPIO      kGates[4];

// ------------------- Gate Helpers -------------------

static void UpdateGates()
{
    bool any_gate_active = false;
    for(int i = 0; i < 4; ++i)
    {
        kGates[i].Write(voices[i].gate);
        if(voices[i].gate) any_gate_active = true;
    }
    hw.SetLed(any_gate_active);
}

static inline int FindFreeVoice()
{
    for(int v = 0; v < kNumVoices; ++v)
        if(!voices[v].gate) return v;
    return -1;
}

static int FindOldestActiveVoice()
{
    int oldest_vi = -1;
    uint32_t oldest_ts = 0;
    for(int v=0; v<kNumVoices; ++v)
    {
        if(!voices[v].gate) continue;
        const int n = voices[v].note;
        const uint32_t ts = midi_hold_ts[n];
        if(oldest_vi < 0 || ts < oldest_ts) { oldest_ts = ts; oldest_vi = v; }
    }
    return oldest_vi;
}

static void AssignVoiceToMidi(int voice_idx, int note)
{
    int old_note = voices[voice_idx].note;
    if(old_note >= 0 && midi_voice[old_note] == voice_idx) {
        midi_voice[old_note] = -1;
    }

    voices[voice_idx].active = true;
    voices[voice_idx].note   = note;
    voices[voice_idx].gate   = true; // PIN HIGH
    midi_voice[note] = voice_idx;
}

static void ReleaseVoice(int voice_idx)
{
    const int note = voices[voice_idx].note;
    if(note >= 0 && midi_voice[note] == voice_idx)
        midi_voice[note] = -1;

    voices[voice_idx].gate = false; // PIN LOW
}

static void OnMidiNoteOn(int note)
{
    if(note < 0 || note >= kNumMidiNotes) return;
    midi_held[note] = true;
    midi_hold_ts[note] = ++global_press_counter;

    int free_v = FindFreeVoice();
    if(free_v >= 0)
    {
        AssignVoiceToMidi(free_v, note);
        UpdateGates();
        return;
    }

    int steal_vi = FindOldestActiveVoice();
    if(steal_vi < 0) steal_vi = 0; 

    AssignVoiceToMidi(steal_vi, note);
    UpdateGates();
}

static void OnMidiNoteOff(int note)
{
    if(note < 0 || note >= kNumMidiNotes) return;
    midi_held[note] = false;
    int owned_v = midi_voice[note];
    if(owned_v >= 0)
        ReleaseVoice(owned_v);

    UpdateGates();
}

// ===== Pitch Bend Handler =====
static void HandlePitchBend(MidiEvent m)
{
    // PitchBend is 14-bit: Data1 (LSB) + Data2 (MSB)
    // Value range: 0 to 16383. Center (no bend) is 8192.
    uint16_t val = (m.data[1] << 7) | m.data[0];
    
    // Normalize to -1.0 to 1.0
    float norm = (float)(val - 8192) / 8192.0f; 

    // Set Bend Range: +/- 2 Semitones
    float semitones = norm * 2.0f; 

    // Convert semitones to frequency multiplier: 2^(semitones/12)
    g_bend_mult = powf(2.0f, semitones / 12.0f);
}

// ===== MIDI event handling =====
static void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
      OnMidiNoteOn(m.data[0]);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
      OnMidiNoteOff(m.data[0]);
    else if(m.type == PitchBend) // Handle Pitch Bend Events
      HandlePitchBend(m);
}

// ===== Audio Callback =====
static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    const float cents1 = (detune1 - 0.5f) * 400.0f;
    const float cents2 = (detune2 - 0.5f) * 400.0f;
    const float detuneFactor1 = powf(2.0f, cents1 / 1200.0f);
    const float detuneFactor2 = powf(2.0f, cents2 / 1200.0f);
    const float mix_scale = 1.f / (2.f * kNumVoices);
    
    for(int v = 0; v < kNumVoices; ++v)
    {
        int note = voices[v].note >= 0 ? voices[v].note : 60;
        const float f = mtof(static_cast<float>(note));

        // Apply g_bend_mult to the frequency
        osc1[v].SetFreq(f * detuneFactor1 * g_bend_mult);
        osc2[v].SetFreq(f * detuneFactor2 * g_bend_mult);

        osc1[v].SetAmp(volume1); 
        osc1[v].SetPw(pulseW1);

        osc2[v].SetAmp(volume2); 
        osc2[v].SetPw(pulseW2);
    }

    for(size_t i = 0; i < size; ++i)
    {
        float mix[kNumVoices];
        for(int v = 0; v < kNumVoices; ++v)
        {
            if(voices[v].active)
                mix[v] = osc1[v].Process() + osc2[v].Process();
            else 
                mix[v] = 0.0f;
            mix[v] *= mix_scale; 
        }
       
        out[0][i] = mix[0];
        out[1][i] = mix[1]; 
        out[2][i] = mix[3]; 
        out[3][i] = mix[2]; 
    }
}

// -------------------------- CRC8 -------------------------------
uint8_t crc8(const uint8_t* d, int n)
{
    uint8_t c = 0;
    for(int i = 0; i < n; i++) {
        c ^= d[i];
        for(int b = 0; b < 8; b++)   
            c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1); 
    }
    return c;
}

static inline float fmap_range(uint16_t v, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
    if(in_max == in_min) return out_min;
    float t = (float)(v - in_min) / (float)(in_max - in_min);
    if(t < 0.0f) t = 0.0f;
    if(t > 1.0f) t = 1.0f;
    return out_min + t * (out_max - out_min);
}

void ApplyParameters(uint16_t* vals, int n)
{
    if(n < 11) return;

    volume1 = fmap_range(vals[0], 0, 65535, 0.0f, 1.0f);
    pulseW1 = fmap_range(vals[1], 0, 65535, 0.0f, 1.0f); 
    detune1 = fmap_range(vals[2], 0, 65535, 0.0f, 1.0f);
    float waveSel1 = fmap_range(vals[3], 0, 65535, 0.0f, 7.99f); 

    volume2 = fmap_range(vals[4], 0, 65535, 0.0f, 1.0f);
    pulseW2 = fmap_range(vals[5], 0, 65535, 0.0f, 1.0f);  
    detune2 = fmap_range(vals[6], 0, 65535, 0.0f, 1.0f);
    float waveSel2 = fmap_range(vals[7], 0, 65535, 0.0f, 7.99f); 

    int waveIndex1 = static_cast<int>(waveSel1);
    int waveIndex2 = static_cast<int>(waveSel2);
    for(int v = 0; v < kNumVoices; ++v)
    {
        switch(waveIndex1) {
            case 0: osc1[v].SetWaveform(Oscillator::WAVE_SIN); break;
            case 1: osc1[v].SetWaveform(Oscillator::WAVE_TRI); break;
            case 2: osc1[v].SetWaveform(Oscillator::WAVE_SAW); break;
            case 3: osc1[v].SetWaveform(Oscillator::WAVE_RAMP); break;
            case 4: osc1[v].SetWaveform(Oscillator::WAVE_SQUARE); break;
            case 5: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
            case 6: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
            case 7: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        }
        switch(waveIndex2) {
            case 0: osc2[v].SetWaveform(Oscillator::WAVE_SIN); break;
            case 1: osc2[v].SetWaveform(Oscillator::WAVE_TRI); break;
            case 2: osc2[v].SetWaveform(Oscillator::WAVE_SAW); break;
            case 3: osc2[v].SetWaveform(Oscillator::WAVE_RAMP); break;
            case 4: osc2[v].SetWaveform(Oscillator::WAVE_SQUARE); break;
            case 5: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
            case 6: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
            case 7: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        }
    }
}

static void TryApplyAll()
{
    if(g_ctrl_parts == 0x0F) 
    {
        ApplyParameters(g_ctrl_buf, 11);
        g_ctrl_parts = 0; 
    }
}

void ProcessFrame(uint8_t* data, int n)
{
    if(n < 1 + 1 + 4 + 1) return;
    int p = 0;
    uint8_t len  = data[p++]; 
    uint8_t type = data[p++]; 
    p += 4; // skip seq

    int param_bytes = len - (1 + 4);
    int nparams = param_bytes / 2;
    if(nparams <= 0) return;

    switch(type)
    {
        case 0x01: 
            for(int i = 0; i < nparams && i < 3; i++) {
                g_ctrl_buf[0 + i] = data[p] | (data[p+1] << 8);
                p += 2;
            }
            g_ctrl_parts |= 0x01;
            break;
        case 0x02: 
            for(int i = 0; i < nparams && i < 3; i++) {
                g_ctrl_buf[3 + i] = data[p] | (data[p+1] << 8);
                p += 2;
            }
            g_ctrl_parts |= 0x02;
            break;
        case 0x03: 
            for(int i = 0; i < nparams && i < 3; i++) {
                g_ctrl_buf[6 + i] = data[p] | (data[p+1] << 8);
                p += 2;
            }
            g_ctrl_parts |= 0x04;
            break;
        case 0x04: 
            for(int i = 0; i < nparams && i < 2; i++) {
                g_ctrl_buf[9 + i] = data[p] | (data[p+1] << 8);
                p += 2;
            }
            g_ctrl_parts |= 0x08;
            break;
    }
    TryApplyAll();
}

void ParseByte(uint8_t b)
{
    switch(state)
    {
        case 0: if(b == HEADER1) state = 1; break;
        case 1: if(b == HEADER2) state = 2; else state = 0; break;
        case 2: 
            expectedLen = b + 1;
            rxIndex = 0;
            state = 3;
            break;
        case 3: 
            rxBuf[rxIndex++] = b;
            if(rxIndex >= expectedLen)
            {
                uint8_t crc_recv = rxBuf[expectedLen - 1];
                uint8_t crc_calc = crc8(rxBuf, expectedLen - 1); 
                if(crc_recv == crc_calc)
                    ProcessFrame(rxBuf, expectedLen - 1);
                state   = 0;
                rxIndex = 0;
            }
            break;
    }
}

void ReceiveLoop()
{
    uint8_t b;
    while(uart.BlockingReceive(&b, 1, 0) == UartHandler::Result::OK)
    {
        ParseByte(b);
    }
}

void InitUart()
{
    UartHandler::Config cfg;
    cfg.periph = UartHandler::Config::Peripheral::UART_4; 
    cfg.mode   = UartHandler::Config::Mode::TX_RX; 
    cfg.baudrate = 76800;
    cfg.pin_config.rx = D11; 
    cfg.pin_config.tx = D12; 
    uart.Init(cfg);
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(48);
    
    // 1. Initialize MIDI (activates USART1)
    {
        MidiUartHandler::Config midi_cfg;
        midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
        midi_cfg.transport_config.rx = D30;
        midi_cfg.transport_config.tx = D29; 
        midi.Init(midi_cfg);
    }

    // 2. Initialize GATES (D15-D18)
    // CRITICAL: Run AFTER MIDI init to reclaim D15 as GPIO Output
    for(int i = 0; i < 4; ++i)
    {
        kGates[i].Init(kGatePins[i], GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
        kGates[i].Write(false); 
    }

    // 3. Oscillators
    for(int v=0; v<kNumVoices; ++v)
    {
        osc1[v].Init(hw.AudioSampleRate());
        osc2[v].Init(hw.AudioSampleRate());
        osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        osc1[v].SetAmp(1.0f); 
        osc2[v].SetAmp(1.0f);
    }

    for(int n = 0; n < kNumMidiNotes; ++n)
    {
        midi_voice[n]   = -1;
        midi_hold_ts[n] = 0;
        midi_held[n]    = false;
    }
    UpdateGates();
    
    // 4. SAI2 (External PCM3060) config
    SaiHandle         sai2;
    SaiHandle::Config sc;
    sc.periph    = SaiHandle::Config::Peripheral::SAI_2;
    sc.sr        = SaiHandle::Config::SampleRate::SAI_48KHZ;
    sc.bit_depth = SaiHandle::Config::BitDepth::SAI_24BIT;
    sc.a_sync    = SaiHandle::Config::Sync::SLAVE;   
    sc.b_sync    = SaiHandle::Config::Sync::MASTER;
    sc.a_dir     = SaiHandle::Config::Direction::TRANSMIT;   
    sc.b_dir     = SaiHandle::Config::Direction::RECEIVE; 
    sc.pin_config.mclk = D24;
    sc.pin_config.sck  = D28; 
    sc.pin_config.fs   = D27; 
    sc.pin_config.sa   = D26; 
    sc.pin_config.sb   = D25; 
    sai2.Init(sc);
    
    // 5. UART and Audio Start
    InitUart();      

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 48; 
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 0.5f;

    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    hw.StartAudio(AudioCallback);
    
    // 6. Main Loop
    while(true)
    {
        midi.Listen();
        while(midi.HasEvents())
            HandleMidiMessage(midi.PopEvent());

        ReceiveLoop();
    }
}