#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"
#include <cstring> 
#include <cmath>

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ==========================================
// 1. HARDWARE & GLOBALS
// ==========================================
DaisySeed hw;
static constexpr int kNumVoices = 4; 

UartHandler uart;

// UART Buffer
uint8_t uart_rx_buff[18]; 
uint32_t last_knob_update = 0;

// Defaults
uint16_t last_knob_values[8] = {
    0x0000, 0x8000, 0x8000, 0xFFFF, 
    0x0000, 0x8000, 0x8000, 0xFFFF
}; 

// Params
float volume1 = 1.0f, volume2 = 1.0f; 
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune1 = 0.5f, detune2 = 0.5f;
int waveIndex1 = 0;
int waveIndex2 = 0;
float g_bend_mult = 1.0f; 

// THREAD SAFETY VARIABLES
volatile int target_wave_idx_1 = 0;
volatile int target_wave_idx_2 = 0;

int current_wave_idx_1 = -1;
int current_wave_idx_2 = -1;

Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];
WhiteNoise white_noise[kNumVoices]; 

// Anti-Aliasing Filter
float smooth_mem[kNumVoices];

MidiUartHandler midi;

struct Voice {
    bool     active = false; 
    bool     gate   = false; 
    int      note   = -1;    
};
Voice voices[kNumVoices];

static constexpr int kNumMidiNotes = 128;
int      midi_voice[kNumMidiNotes];        
uint32_t midi_hold_ts[kNumMidiNotes];      
uint32_t global_press_counter = 0;         

static const Pin kGatePins[4] = { D15, D16, D17, D18 };
static GPIO      kGates[4];

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================
static void UpdateGates()
{
    bool any_gate_active = false;
    for(int i = 0; i < 4; ++i) {
        kGates[i].Write(voices[i].gate);
        if(voices[i].gate) any_gate_active = true;
    }
    hw.SetLed(any_gate_active);
}

static inline int FindFreeVoice()
{
    for(int v = 0; v < kNumVoices; ++v)
        if(!voices[v].gate && !voices[v].active) return v;
    for(int v = 0; v < kNumVoices; ++v)
        if(!voices[v].gate) return v;
    return -1;
}

static int FindOldestActiveVoice()
{
    int oldest_vi = -1;
    uint32_t oldest_ts = 0xFFFFFFFF;
    for(int v=0; v<kNumVoices; ++v) {
        if(!voices[v].gate) continue;
        const int n = voices[v].note;
        const uint32_t ts = midi_hold_ts[n];
        if(ts < oldest_ts) { oldest_ts = ts; oldest_vi = v; }
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
    midi_hold_ts[note] = ++global_press_counter;

    int free_v = FindFreeVoice();
    if(free_v >= 0) {
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
    int owned_v = midi_voice[note];
    if(owned_v >= 0) ReleaseVoice(owned_v);
    UpdateGates();
}

static void HandlePitchBend(MidiEvent m)
{
    uint16_t val = (m.data[1] << 7) | m.data[0];
    float norm = (float)(val - 8192) / 8192.0f; 
    g_bend_mult = powf(2.0f, (norm * 2.0f) / 12.0f);
}

static void HandleMidiMessage(MidiEvent m)
{
    if(m.type == NoteOn && m.data[1] > 0) OnMidiNoteOn(m.data[0]);
    else if(m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0)) OnMidiNoteOff(m.data[0]);
    else if(m.type == PitchBend) HandlePitchBend(m);
}

// ==========================================
// 3. FAST SINE LUT
// ==========================================
static float sine_lut[2048];

void InitSineLut() {
    for(int i = 0; i < 2048; ++i) {
        sine_lut[i] = sinf(i * TWOPI_F / 2048.0f);
    }
}

static inline float FastSin(float rad) {
    float v = (rad * 325.94932f) + 1024000.0f; 
    int i = static_cast<int>(v);
    float frac = v - i;
    float a = sine_lut[i & 2047];
    float b = sine_lut[(i + 1) & 2047];
    return a + (b - a) * frac;
}

// ==========================================
// 4. AUDIO GENERATION
// ==========================================

void UpdateOscillators() {
    if (target_wave_idx_1 == current_wave_idx_1 && target_wave_idx_2 == current_wave_idx_2) return;

    current_wave_idx_1 = target_wave_idx_1;
    current_wave_idx_2 = target_wave_idx_2;

    for(int v = 0; v < kNumVoices; ++v) {
        // OSC 1
        int w1 = current_wave_idx_1;
        if (w1 <= 3) {
            if(w1==0) osc1[v].SetWaveform(Oscillator::WAVE_SIN);
            else if(w1==1) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
            else if(w1==2) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
            else osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        } else if (w1 == 4) osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
        else if (w1 == 11) osc1[v].SetWaveform(Oscillator::WAVE_SIN);
        else osc1[v].SetWaveform(Oscillator::WAVE_RAMP);

        // OSC 2
        int w2 = current_wave_idx_2;
        if (w2 <= 3) {
            if(w2==0) osc2[v].SetWaveform(Oscillator::WAVE_SIN);
            else if(w2==1) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
            else if(w2==2) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
            else osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        } else if (w2 == 4) osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        else if (w2 == 11) osc2[v].SetWaveform(Oscillator::WAVE_SIN);
        else osc2[v].SetWaveform(Oscillator::WAVE_RAMP);
    }
}

static inline float GenerateWave(int mode, Oscillator& osc, WhiteNoise& wn, float pw)
{
    float raw = osc.Process();

    switch(mode) {
        case 0: return FastSin(raw * (1.0f + (pw * 4.0f))); 
        case 1: { 
            float val = raw * (1.0f + (pw * 4.0f));
            return FastSin(val); 
        }
        case 2: return raw; 
        case 3: { 
            float val = raw * (1.0f + (pw * 4.0f));
            return FastSin(val);
        }
        case 4: { // Staircase
             // Exponential curve (pw*pw) keeps steps low/crunchy for longer.
             // 0% = 2 steps. 50% = ~10 steps. 100% = 34 steps.
             float steps = 2.0f + (pw * pw * 32.0f); 
             return floorf(raw * steps) / steps;
        }
        case 5: return FastSin(raw * PI_F * (1.0f + (pw * 14.0f))); 
        case 6: return FastSin((raw * PI_F) + (FastSin(raw * PI_F) * (pw * 4.0f)));
        case 7: return FastSin((raw * PI_F) + (FastSin(raw * 2.0f * PI_F) * (pw * 5.0f)));
        case 8: return FastSin((raw * PI_F) + (FastSin(raw * 1.41f * PI_F) * (pw * 6.0f)));
        case 9: return FastSin((raw * PI_F) + (FastSin(raw * 5.0f * PI_F) * (pw * 3.0f)));
        case 10: return FastSin(raw * PI_F * (1.0f + (pw * 12.0f))) * (1.0f - fabsf(raw));
        case 11: {
            float noise = wn.Process() * (pw * 4.0f);
            return FastSin((raw * PI_F) + noise);
        }
        default: return 0.0f;
    }
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    UpdateOscillators();

    const float pitch1 = powf(2.0f, ((2*detune1) - 1)) * g_bend_mult; 
    const float pitch2 = powf(2.0f, ((2*detune2) - 1)) * g_bend_mult;
    const float mix_scale = 0.25f; 
    
    // Safe Pulse Width: 0.5 (Square) to 0.95 (Thin). Prevents DC silence.
    float safe_pw1 = 0.5f + (pulseW1 * 0.45f); 
    float safe_pw2 = 0.5f + (pulseW2 * 0.45f);

    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active) {
            int note = (voices[v].note >= 0) ? voices[v].note : 60;
            const float f = mtof(static_cast<float>(note));
            osc1[v].SetFreq(f * pitch1);
            osc2[v].SetFreq(f * pitch2);
            osc1[v].SetAmp(1.0f); osc2[v].SetAmp(1.0f);
            
            // Oscillators get Safe PW
            osc1[v].SetPw(safe_pw1);  
            osc2[v].SetPw(safe_pw2);
            
            white_noise[v].SetAmp(1.0f); 
        }
    }

    for(size_t i = 0; i < size; ++i)
    {
        for(int v = 0; v < kNumVoices; ++v) 
        {
            float output_sample = 0.0f;
            
            if(voices[v].active) 
            {
                // GenerateWave gets Raw PW for full effect range
                float sig1 = GenerateWave(current_wave_idx_1, osc1[v], white_noise[v], pulseW1);
                float sig2 = GenerateWave(current_wave_idx_2, osc2[v], white_noise[v], pulseW2);
                output_sample = (sig1 * volume1) + (sig2 * volume2);
                output_sample *= mix_scale;
            }

            switch(v) {
                case 0: out[0][i] = output_sample; break;
                case 1: out[1][i] = output_sample; break;
                case 2: out[3][i] = output_sample; break; 
                case 3: out[2][i] = output_sample; break;
            }
        }
    }
}

// ==========================================
// 5. PARAMETER CONTROL
// ==========================================
static inline float fmap_range(uint16_t v, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
    float t = (float)v * (1.0f / 65535.0f); 
    return out_min + t * (out_max - out_min);
}

void ApplyParameters(uint16_t* vals)
{
    float waveSel1 = fmap_range(vals[0], 0, 65535, 0.0f, 11.99f); 
    detune1        = fmap_range(vals[1], 0, 65535, 0.0f, 1.0f);
    pulseW1        = fmap_range(vals[2], 0, 65535, 0.0f, 1.0f); 
    volume1        = fmap_range(vals[3], 0, 65535, 0.0f, 1.0f);

    float waveSel2 = fmap_range(vals[4], 0, 65535, 0.0f, 11.99f); 
    detune2        = fmap_range(vals[5], 0, 65535, 0.0f, 1.0f);
    pulseW2        = fmap_range(vals[6], 0, 65535, 0.0f, 1.0f);  
    volume2        = fmap_range(vals[7], 0, 65535, 0.0f, 1.0f);
    
    int newW1 = static_cast<int>(waveSel1);
    int newW2 = static_cast<int>(waveSel2);
    
    target_wave_idx_1 = static_cast<int>(waveSel1);
    target_wave_idx_2 = static_cast<int>(waveSel2);
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
// 6. CONTROL LOOP
// ==========================================
void GetKnobs() 
{ 
    uint8_t trash;
    while(uart.BlockingReceive(&trash, 1, 0) == UartHandler::Result::OK);

    uint8_t command = 0x69; 
    uart.BlockingTransmit(&command, 1, 10); 
    
    UartHandler::Result res = uart.BlockingReceive(uart_rx_buff, 18, 40); 

    if(res == UartHandler::Result::OK)
    {
        uint16_t current_vals[8];
        for(uint8_t i = 0; i < 8; i++)
            current_vals[i] = (uart_rx_buff[(2 * i) + 1] << 8) | uart_rx_buff[(2*i) + 2];
        
        // Independent Hysteresis
        bool any_changed = false;
        for(int i=0; i<8; i++) {
            if(abs((int)current_vals[i] - (int)last_knob_values[i]) > 800) {
                last_knob_values[i] = current_vals[i];
                any_changed = true;
            }
        }

        if (any_changed) {
            ApplyParameters(last_knob_values);
        }
    }
}

// ==========================================
// 7. MAIN
// ==========================================
int main(void)
{
    hw.Configure();
    hw.Init();
    
    hw.SetAudioBlockSize(16); 
    
    MidiUartHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
    midi_cfg.transport_config.rx = D30; midi_cfg.transport_config.tx = D29; 
    midi.Init(midi_cfg);

    for(int i = 0; i < 4; ++i) {
        kGates[i].Init(kGatePins[i], GPIO::Mode::OUTPUT, GPIO::Pull::PULLDOWN);
        kGates[i].Write(false); 
    }

    float sr = hw.AudioSampleRate();
    for(int v=0; v<kNumVoices; ++v) {
        osc1[v].Init(sr); osc2[v].Init(sr); white_noise[v].Init();
        osc1[v].SetAmp(1.0f); osc2[v].SetAmp(1.0f);
    }
    for(int n = 0; n < kNumMidiNotes; ++n) {
        midi_voice[n] = -1; midi_hold_ts[n] = 0;
    }
    
    SaiHandle sai2;
    SaiHandle::Config sc;
    sc.periph = SaiHandle::Config::Peripheral::SAI_2;
    sc.sr = SaiHandle::Config::SampleRate::SAI_48KHZ;
    sc.bit_depth = SaiHandle::Config::BitDepth::SAI_24BIT;
    sc.a_sync = SaiHandle::Config::Sync::SLAVE; sc.b_sync = SaiHandle::Config::Sync::MASTER;
    sc.a_dir = SaiHandle::Config::Direction::TRANSMIT; sc.b_dir = SaiHandle::Config::Direction::RECEIVE; 
    sc.pin_config.mclk = D24; sc.pin_config.sck = D28; sc.pin_config.fs = D27; sc.pin_config.sa = D26; sc.pin_config.sb = D25; 
    sai2.Init(sc);

    InitUart();      
    InitSineLut(); 
    ApplyParameters(last_knob_values);

    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 16; 
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 0.3f;
    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    hw.StartAudio(AudioCallback);

    last_knob_update = System::GetNow();

    while(true)
    {
        midi.Listen();
        while(midi.HasEvents()) HandleMidiMessage(midi.PopEvent());

        uint32_t now = System::GetNow();
        if (now - last_knob_update > 40) 
        {
            GetKnobs();
            last_knob_update = now;
        }
    }
}
