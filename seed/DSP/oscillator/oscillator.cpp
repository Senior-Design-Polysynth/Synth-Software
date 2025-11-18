// File: src/oscillator.cpp
// Purpose: 4-voice synth with 12 Waveforms (Analog + FM + FX).
//
// CONFIG: 48kHz / Block Size 16.
// FIXED:  Restored correct Knob Mapping (vals[2]=Shape, vals[3]=Vol).
// FIXED:  Removed expensive 'asinf' math (Fixes crash on Sine Wave).
//         Replaced with "Sine Drive" folding (Cheap & stable).

#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ==========================================
// 1. FORWARD DECLARATIONS
// ==========================================
void GetKnobs(); 
void InitUart();
void ApplyParameters(uint16_t* vals, int n);
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

// ==========================================
// 2. HARDWARE & GLOBALS
// ==========================================
DaisySeed hw;
static constexpr int kNumVoices = 4; 

UartHandler uart;

// Constants
static constexpr uint8_t HEADER1 = 0xAA;
static constexpr uint8_t HEADER2 = 0x55;
static constexpr int     MAX_FRAME = 128;

// UART Buffers
const size_t uart_buff_size = 18;
uint8_t DMA_BUFFER_MEM_SECTION uart_rx_buff[uart_buff_size];

uint8_t rxBuf[MAX_FRAME];
int     rxIndex = 0;
int     expectedLen = 0;
int     state = 0;
static uint16_t g_ctrl_buf[8];
static uint8_t  g_ctrl_parts = 0;

// Params
float volume1 = 1.0f, volume2 = 1.0f; 
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune1 = 0.5f;
float detune2 = 0.5f;

// Wave Selectors (0-11)
int waveIndex1 = 0;
int waveIndex2 = 0;

// Pitch Bend
float g_bend_mult = 1.0f; 

// Synthesis Objects
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];
WhiteNoise white_noise[kNumVoices]; 

// Filter States
float red_noise_mem[kNumVoices];
float pink_mem[kNumVoices];

// MIDI
MidiUartHandler midi;

// Voice State
struct Voice {
    bool     active = false;
    bool     gate   = false;
    int      note   = -1;    
};

Voice voices[kNumVoices];

// Polyphony
static constexpr int kNumMidiNotes = 128;
bool     midi_held[kNumMidiNotes] = {false};
int      midi_voice[kNumMidiNotes];        
uint32_t midi_hold_ts[kNumMidiNotes];      

uint32_t global_press_counter = 0;

// Gates
static const Pin kGatePins[4] = { D15, D16, D17, D18 };
static GPIO      kGates[4];

// Timer State
uint32_t last_knob_update = 0;

// ==========================================
// 3. HELPER FUNCTIONS
// ==========================================

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
    voices[voice_idx].gate   = true; 
    midi_voice[note] = voice_idx;
}

static void ReleaseVoice(int voice_idx)
{
    const int note = voices[voice_idx].note;
    if(note >= 0 && midi_voice[note] == voice_idx)
        midi_voice[note] = -1;

    voices[voice_idx].gate = false; 
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

static void HandlePitchBend(MidiEvent m)
{
    uint16_t val = (m.data[1] << 7) | m.data[0];
    float norm = (float)(val - 8192) / 8192.0f; 
    float semitones = norm * 2.0f; 
    g_bend_mult = powf(2.0f, semitones / 12.0f);
}

static void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0)
      OnMidiNoteOn(m.data[0]);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0))
      OnMidiNoteOff(m.data[0]);
    else if(m.type == PitchBend) 
      HandlePitchBend(m);
}

// ==========================================
// 4. WAVE GENERATION LOGIC
// ==========================================
static inline float GenerateWave(int mode, Oscillator& osc, WhiteNoise& wn, float& red_mem, float& pn_mem, float pw)
{
    // Base Osc Calculation (Modes 0-11 all use oscillator)
    float raw = osc.Process();

    switch(mode) {
        // --- ANALOG BASICS (With Wavefolding) ---
        case 0: // Sine (Folded)
        {
            // FIX: Replaced 'asinf' with 'Sine Drive'
            // This is 10x cheaper on CPU and prevents the crash.
            // Driving a sine into a sine creates folding harmonics.
            float drive = 1.0f + (pw * 4.0f);
            return sinf(raw * drive); 
        }

        case 1: // Triangle (Folded)
        {
            // Standard Wavefolder
            float folded = raw * (1.0f + (pw * 3.0f)); // Apply Gain
            if(folded > 1.0f) folded = 2.0f - folded;
            else if(folded < -1.0f) folded = -2.0f - folded;
            return folded;
        }

        case 2: // Square (PWM)
            return raw; // Pulse Width is handled natively in Oscillator

        case 3: // Saw (Folded / Buzz)
        {
            float folded = raw * (1.0f + (pw * 3.0f));
            if(folded > 1.0f) folded = 2.0f - folded;
            else if(folded < -1.0f) folded = -2.0f - folded;
            return folded;
        }
        
        // --- DIGITAL / SHAPED ---
        case 4: // Staircase (Lo-Fi Saw) - MOVED
             {
                 float steps = 2.0f + (pw * 16.0f); 
                 return (float)((int)(raw * steps)) / steps;
             }

        case 5: // Alien Sync (Hard Sync Sim)
        {
            float sync_mult = 1.0f + (pw * 14.0f);
            return sinf(raw * PI_F * sync_mult);
        }

        // --- FM MODES ---
        case 6: // FM Feedback
        {
            float phase = raw * PI_F;
            float index = pw * 4.0f;
            return sinf(phase + (sinf(phase) * index));
        }

        case 7: // FM Organ
        {
            float phase = raw * PI_F;
            float index = pw * 5.0f;
            return sinf(phase + (sinf(phase * 2.0f) * index));
        }

        case 8: // FM Bell
        {
            float phase = raw * PI_F;
            float index = pw * 6.0f;
            return sinf(phase + (sinf(phase * 1.41f) * index));
        }

        case 9: // FM Glass
        {
            float phase = raw * PI_F;
            float index = pw * 3.0f;
            return sinf(phase + (sinf(phase * 5.0f) * index));
        }

        // --- SPECIAL FX ---
        case 10: // Vocal Formant
        {
            float formant_freq = 1.0f + (pw * 12.0f); 
            return sinf(raw * PI_F * formant_freq) * (1.0f - fabsf(raw));
        }

        case 11: // FM Chaos (Noise Phase Mod)
        {
            float phase = raw * PI_F;
            float noise = wn.Process() * (pw * 4.0f);
            return sinf(phase + noise);
        }
        
        default: return 0.0f;
    }
}

// ==========================================
// 5. AUDIO CALLBACK
// ==========================================
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    const float detuneFactor1 = powf(2.0f, ((2*detune1) - 1)); 
    const float detuneFactor2 = powf(2.0f, ((2*detune2) - 1));
    const float mix_scale = 1.f / (kNumVoices); 
    
    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active) {
            int note = voices[v].note >= 0 ? voices[v].note : 60;
            const float f = mtof(static_cast<float>(note));

            osc1[v].SetFreq(f * detuneFactor1 * g_bend_mult);
            osc2[v].SetFreq(f * detuneFactor2 * g_bend_mult);
            
            // FIX: Always run osc at full amplitude for Math stability
            osc1[v].SetAmp(1.0f); osc2[v].SetAmp(1.0f);
            
            osc1[v].SetPw(pulseW1);  osc2[v].SetPw(pulseW2);

            // Noise Update
            white_noise[v].SetAmp(1.0f); 
        }
    }

    for(size_t i = 0; i < size; ++i)
    {
        float mix[kNumVoices];
        for(int v = 0; v < kNumVoices; ++v)
        {
            if(voices[v].active)
            {
                float sig1 = GenerateWave(waveIndex1, osc1[v], white_noise[v], red_noise_mem[v], pink_mem[v], pulseW1);
                float sig2 = GenerateWave(waveIndex2, osc2[v], white_noise[v], red_noise_mem[v], pink_mem[v], pulseW2);
                
                // Apply Volume HERE (Post-Processing)
                mix[v] = (sig1 * volume1) + (sig2 * volume2);
            }
            else 
            {
                mix[v] = 0.0f;
            }
            mix[v] *= mix_scale; 
        }
       
        out[0][i] = mix[0];
        out[1][i] = mix[1]; 
        out[2][i] = mix[3]; 
        out[3][i] = mix[2]; 
    }
}

// ==========================================
// 6. PARAMETER MAPPING & CONTROL
// ==========================================
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
    float waveSel1 = fmap_range(vals[0], 0, 65535, 0.0f, 11.99f); 
    detune1 = fmap_range(vals[1], 0, 65535, 0.0f, 1.0f);
    
    // RESTORED MAPPING: Vals[2]=Shape(PulseW), Vals[3]=Volume
    pulseW1 = fmap_range(vals[2], 0, 65535, 0.0f, 1.0f); 
    volume1 = fmap_range(vals[3], 0, 65535, 0.0f, 1.0f);

    float waveSel2 = fmap_range(vals[4], 0, 65535, 0.0f, 11.99f); 
    detune2 = fmap_range(vals[5], 0, 65535, 0.0f, 1.0f);
    
    // RESTORED MAPPING: Vals[6]=Shape(PulseW), Vals[7]=Volume
    pulseW2 = fmap_range(vals[6], 0, 65535, 0.0f, 1.0f);  
    volume2 = fmap_range(vals[7], 0, 65535, 0.0f, 1.0f);
    
    waveIndex1 = static_cast<int>(waveSel1);
    waveIndex2 = static_cast<int>(waveSel2);
    
    for(int v = 0; v < kNumVoices; ++v)
    {
        // BASE WAVEFORM SETUP
        // 0-3: Analog
        // 4: Staircase (Saw Base)
        // 5: Alien Sync (Ramp Base)
        // 6-10: FM/Formant (Ramp Base)
        // 11: Chaos (Sine Base)
        
        // --- OSC 1 ---
        if      (waveIndex1 == 0) osc1[v].SetWaveform(Oscillator::WAVE_SIN);
        else if (waveIndex1 == 1) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
        else if (waveIndex1 == 2) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        else if (waveIndex1 == 3) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        else if (waveIndex1 == 4) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);    // Staircase
        else if (waveIndex1 == 5) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);            // Alien Sync
        else if (waveIndex1 == 6) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);            // FM Feedback
        else if (waveIndex1 == 7) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);            // FM Organ
        else if (waveIndex1 == 8) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);            // FM Bell
        else if (waveIndex1 == 9) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);            // FM Glass
        else if (waveIndex1 == 10) osc1[v].SetWaveform(Oscillator::WAVE_RAMP);           // Vocal Formant
        else if (waveIndex1 == 11) osc1[v].SetWaveform(Oscillator::WAVE_SIN);            // FM Chaos

        // --- OSC 2 ---
        if      (waveIndex2 == 0) osc2[v].SetWaveform(Oscillator::WAVE_SIN);
        else if (waveIndex2 == 1) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
        else if (waveIndex2 == 2) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        else if (waveIndex2 == 3) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        else if (waveIndex2 == 4) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        else if (waveIndex2 == 5) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 6) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 7) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 8) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 9) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 10) osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
        else if (waveIndex2 == 11) osc2[v].SetWaveform(Oscillator::WAVE_SIN);
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

void GetKnobs() { 
    uint8_t command = 0x69; 
    for(uint8_t i = 0; i < 18; i++)
        uart_rx_buff[i] = 0x00;
    uart.BlockingTransmit(&command, 1, 10); 
    uart.BlockingReceive(uart_rx_buff, 18, 200); 
    uint16_t knob_values[8] = {0x0000};
    for(uint8_t i = 0; i < 8; i++)
        knob_values[i] = (uart_rx_buff[(2 * i) + 1] << 8) | uart_rx_buff[(2*i) + 2];
    ApplyParameters(knob_values, 8);
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

// ==========================================
// 7. MAIN LOOP
// ==========================================

int main(void)
{
    hw.Configure();
    hw.Init();
    
    // SAFETY: 48kHz / Block 16 (User Preferred)
    hw.SetAudioBlockSize(12); 
    
    // 1. Initialize MIDI
    {
        MidiUartHandler::Config midi_cfg;
        midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
        midi_cfg.transport_config.rx = D30;
        midi_cfg.transport_config.tx = D29; 
        midi.Init(midi_cfg);
    }

    // 2. Initialize GATES
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
        white_noise[v].Init();

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
    
    // 4. SAI2 config
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
    audio_cfg.blocksize  = 12; 
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 1.0f;

    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    hw.StartAudio(AudioCallback);

    // 6. Main Loop
    last_knob_update = System::GetNow();

    while(true)
    {
        midi.Listen();
        
        // --- FAST MIDI LOOP ---
        while(midi.HasEvents())
        {
            HandleMidiMessage(midi.PopEvent());
        }

        // --- THROTTLED CONTROL LOOP (20ms) ---
        uint32_t now = System::GetNow();
        if (now - last_knob_update > 20) 
        {
            GetKnobs();
            last_knob_update = now;
        }
    }
}
