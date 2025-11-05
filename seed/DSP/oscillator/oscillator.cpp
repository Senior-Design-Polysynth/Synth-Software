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

UartHandler uart;


// ------------------- Constants for framing -------------------
static constexpr uint8_t HEADER1 = 0xAA;   // first header byte (start of frame)
static constexpr uint8_t HEADER2 = 0x55;   // second header byte
static constexpr int     MAX_FRAME = 128;  // max size of a single incoming frame (bytes)

// ------------------- Receive buffer variables -------------------
uint8_t rxBuf[MAX_FRAME];   // stores the bytes of one complete frame
int     rxIndex = 0;        // how many bytes have been collected so far
int     expectedLen = 0;    // total number of bytes expected in this frame (from 'len' byte)
int     state = 0;          // current parser state (0 = wait for 0xAA, 1 = wait for 0x55, 2 = length, 3 = collecting)

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

// -------------------------- CRC8 -------------------------------
// ===============================================================

// Calculates an 8-bit CRC (Cyclic Redundancy Check) for error detection
// Uses polynomial 0x31 (x^8 + x^5 + x^4 + 1)
uint8_t crc8(const uint8_t* d, int n)
{
    uint8_t c = 0;
    for(int i = 0; i < n; i++)   // iterate through all bytes
    {
        c ^= d[i];               // XOR current byte into accumulator
        for(int b = 0; b < 8; b++)   // process each bit
            c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1); // shift + polynomial feedback
    }
    return c; // final CRC checksum
}

// ---------- Apply received parameters to your DSP --------------
// ===============================================================

// Simple helper to map a 16-bit integer in [in_min,in_max] to a float in [out_min,out_max]
static inline float fmap_range(uint16_t v, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
    if(in_max == in_min) return out_min;
    float t = (float)(v - in_min) / (float)(in_max - in_min);
    if(t < 0.0f) t = 0.0f;
    if(t > 1.0f) t = 1.0f;
    return out_min + t * (out_max - out_min);
}

// -----------------------------------------------------------------------------
// ApplyParameters()
// Maps the 11 UART-received control values (0–65535) into synth parameters.
// Includes 8-way rotary selectors for oscillator waveform selection.
// -----------------------------------------------------------------------------
void ApplyParameters(uint16_t* vals, int n)
{
    if(n < 11)
        return;

    // --- Oscillator 1 parameters ---
    volume1 = fmap_range(vals[0], 0, 65535, 0.0f, 1.0f);  // OSC1 volume
    pulseW1 = fmap_range(vals[1], 0, 65535, 0.0f, 1.0f);  // OSC1 pulse width
    detune1 = fmap_range(vals[2], 0, 65535, 0.0f, 1.0f);  // OSC1 detune
    float waveSel1 = fmap_range(vals[3], 0, 65535, 0.0f, 7.99f); // 8-way rotary (0–7)

    // --- Oscillator 2 parameters ---
    volume2 = fmap_range(vals[4], 0, 65535, 0.0f, 1.0f);  // OSC2 volume
    pulseW2 = fmap_range(vals[5], 0, 65535, 0.0f, 1.0f);  // OSC2 pulse width
    detune2 = fmap_range(vals[6], 0, 65535, 0.0f, 1.0f);  // OSC2 detune
    float waveSel2 = fmap_range(vals[7], 0, 65535, 0.0f, 7.99f); // 8-way rotary (0–7)

    // --- Reserved for expansion ---
    float unused1 = fmap_range(vals[8], 0, 65535, 0.0f, 1.0f);
    float unused2 = fmap_range(vals[9], 0, 65535, 0.0f, 1.0f);
    float unused3 = fmap_range(vals[10], 0, 65535, 0.0f, 1.0f);
    (void)unused1; (void)unused2; (void)unused3;

    // --- Quantize the 8-way selectors to integer 0–7 ---
    int waveIndex1 = static_cast<int>(waveSel1);
    int waveIndex2 = static_cast<int>(waveSel2);

    // --- Assign oscillator waveforms dynamically ---
    for(int v = 0; v < kNumVoices; ++v)
    {
        switch(waveIndex1)
        {
            case 0: osc1[v].SetWaveform(Oscillator::WAVE_SIN);              break;
            case 1: osc1[v].SetWaveform(Oscillator::WAVE_TRI);              break;
            case 2: osc1[v].SetWaveform(Oscillator::WAVE_SAW);              break;
            case 3: osc1[v].SetWaveform(Oscillator::WAVE_RAMP);             break;
            case 4: osc1[v].SetWaveform(Oscillator::WAVE_SQUARE);           break;
            case 5: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);  break;
            case 6: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);     break;
            case 7: osc1[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);     break;
            default: break;
        }

        switch(waveIndex2)
        {
            case 0: osc2[v].SetWaveform(Oscillator::WAVE_SIN);              break;
            case 1: osc2[v].SetWaveform(Oscillator::WAVE_TRI);              break;
            case 2: osc2[v].SetWaveform(Oscillator::WAVE_SAW);              break;
            case 3: osc2[v].SetWaveform(Oscillator::WAVE_RAMP);             break;
            case 4: osc2[v].SetWaveform(Oscillator::WAVE_SQUARE);           break;
            case 5: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);  break;
            case 6: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);     break;
            case 7: osc2[v].SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);     break;
            default: break;
        }
    }
}

// ------------------- Full frame processor ----------------------
// ===============================================================

// Called when an entire frame has been received and verified
void ProcessFrame(uint8_t* data, int n)
{
    // Sanity check: must at least fit header + sequence + one parameter
    if(n < 7)
        return;

    // Compute and verify CRC
    uint8_t crc_recv = data[n - 1];          // last byte of frame = received CRC
    uint8_t crc_calc = crc8(data, n - 1);    // recalc CRC for comparison
    if(crc_recv != crc_calc)
        return; // CRC mismatch → discard frame

    int p = 0;                 // parsing pointer index
    uint8_t len  = data[p++];  // payload length byte
    uint8_t type = data[p++];  // frame type (0x01 for control)

    // Read 4-byte sequence number (little-endian)
    uint32_t seq = data[p]
                 | (data[p+1] << 8)
                 | (data[p+2] << 16)
                 | (data[p+3] << 24);
    p += 4;
    (void)seq;  // currently unused, but can be logged/debugged later

    // Extract 11 × 16-bit parameter values from the data section
    const int nParams = 11;
    uint16_t params[nParams];
    for(int i = 0; i < nParams; i++)
    {
        // combine two bytes into one 16-bit value (little-endian)
        params[i] = data[p] | (data[p + 1] << 8);
        p += 2;
    }

    // Once unpacked, send them to ApplyParameters()
    ApplyParameters(params, nParams);
}

// -------------------- Byte-by-byte parser ----------------------
// ===============================================================

// This is the state machine that reads the raw UART bytes one by one
// It looks for the header (0xAA 0x55), then reads 'len' and collects the frame
void ParseByte(uint8_t b)
{
    switch(state)
    {
        case 0: // Waiting for first header byte (0xAA)
            if(b == HEADER1)
            {     
                state = 1;
            }
            break;

        case 1: // Waiting for second header byte (0x55)
            if(b == HEADER2)
            {
                state = 2;
            }
            else
            {
                state = 0;  // False alarm → restart search
            }
            break;

        case 2: // Reading the 'length' byte (payload size)
            expectedLen = b + 1;
            rxIndex = 0;
            state = 3;
            break;

        case 3: // Collecting remaining bytes
            rxBuf[rxIndex++] = b;
            if(rxIndex >= expectedLen)
            {
                ProcessFrame(rxBuf, rxIndex);
                state = 0;
            }
            break;
    }
}


// ------------------ Continuous UART read loop -----------------------
// ===============================================================

// Polls the UART FIFO and feeds new bytes into the parser
void ReceiveLoop()
{
    uint8_t byte;
    // Try to read one byte (1 ms timeout)
    if(uart.BlockingReceive(&byte, 1, 1) == UartHandler::Result::OK)
    {
        ParseByte(byte);
    }
}

// ---------------------- UART Initialization --------------------
// ===============================================================

// Configure and start UART1 on the DaisySeed
void InitUart()
{
   UartHandler::Config cfg;
    cfg.periph = UartHandler::Config::Peripheral::UART_4;
    cfg.mode   = UartHandler::Config::Mode::RX; // RX only
    cfg.baudrate = 115200;
    cfg.pin_config.rx = {DSY_GPIOB, 8};  // D11
    cfg.pin_config.tx = {DSY_GPIOB, 9};  // D12 (not used)

    uart.Init(cfg);


}

int main(void)
{
    hw.Configure();
    hw.Init();


    hw.SetAudioBlockSize(48); // 1ms @ 48kHz; multiple of 4

    // MIDI UART (DIN)
    {
        MidiUartHandler::Config midi_cfg;
        midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
        midi_cfg.transport_config.rx = D30;
        midi_cfg.transport_config.tx = D29; // not required for input-only
        midi.Init(midi_cfg);
    }

    // Oscillators
    for(int v=0; v<kNumVoices; ++v)
    {
        osc1[v].Init(hw.AudioSampleRate());
        osc2[v].Init(hw.AudioSampleRate());
        osc1[v].SetWaveform(Oscillator::WAVE_SIN);
        osc2[v].SetWaveform(Oscillator::WAVE_SIN);
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

    //======UART Initialization=================
    InitUart();      // configure UART1 (for control data)

    // ==========Dual-SAI (internal + external)==========
    AudioHandle::Config audio_cfg;
    audio_cfg.blocksize  = 48; // multiple of 4
    audio_cfg.samplerate = SaiHandle::Config::SampleRate::SAI_48KHZ;
    audio_cfg.postgain   = 0.5f;

    hw.audio_handle.Init(audio_cfg, hw.AudioSaiHandle(), sai2);
    hw.StartAudio(AudioCallback);

    // Main loop
    while(true)
    {
        // MIDI processing
        midi.Listen();
        while(midi.HasEvents())
          HandleMidiMessage(midi.PopEvent());

        ReceiveLoop(); // check for and parse UART frames

        System::Delay(1);
    }
}
