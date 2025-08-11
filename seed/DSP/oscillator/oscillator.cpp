// Purpose: Two-voice, two-osc-per-voice synth driven ONLY by hardware buttons.
// Behavior:
//  - Up to 2 simultaneous button notes => up to 2 voices (each voice uses 2 oscillators).
//  - On third (or more) press while 2 voices active: steal the OLDEST held button's voice
//    and give it to the newest press.
//  - If the stolen button is still held when the newer button releases, the voice
//    is returned to the stolen (oldest) button. (Fair restitution.)
// Why: Implements deterministic voice allocation with proper restitution; fixes ADC
//      config count, removes MIDI, and avoids reassign-while-scanning races.

#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

// ===== Hardware =====
DaisySeed hw;
static constexpr int kNumVoices = 2;
static constexpr int kNumKeys   = 6;               // 6 hardware buttons
static const Pin kButtonPins[kNumKeys] = {D9, D10, D11, D12, D13, D14};
GPIO keybutton[kNumKeys];

// ===== ADC knobs (5 used here) =====
// 0: OSC1 Volume, 1: OSC1 Pulse Width, 2: (unused), 3: OSC2 Volume, 4: OSC2 Pulse Width, 5: OSC2 Detune
AdcChannelConfig adc_cfg[6];

// ===== Params read each audio block =====
float volume1 = 0.f, volume2 = 0.f;
float pulseW1 = 0.5f, pulseW2 = 0.5f;
float detune  = 0.5f; // 0..1 => -50..+50 cents

// ===== Synthesis =====
Oscillator osc1[kNumVoices];
Oscillator osc2[kNumVoices];

// Map 6 local buttons to MIDI notes then to Hz
static inline float KeyNoteFreq(int btn_id)
{
    static const float kMidiNotes[kNumKeys] = {60, 62, 64, 65, 67, 69}; // C4 D4 E4 F4 G4 A4
    return mtof(kMidiNotes[btn_id]);
}

// ===== Voice/Allocation State =====
struct Voice {
    bool     active = false;
    int      btn_id = -1;       // which button owns this voice
    uint32_t timestamp = 0;     // when assigned (press order)
};

Voice voices[kNumVoices];

// Per-button state
bool     btn_held[kNumKeys] = {false};
int      btn_voice[kNumKeys];     // -1 if none, else voice index
uint32_t btn_hold_ts[kNumKeys];   // press order counter

uint32_t global_press_counter = 0; // increases on each **new** press edge

// Utility: find free voice index or -1
static int FindFreeVoice()
{
    for(int v = 0; v < kNumVoices; ++v)
        if(!voices[v].active) return v;
    return -1;
}

// Utility: among ACTIVE voices, return index whose owning button has the OLDEST hold timestamp
static int FindOldestActiveVoice()
{
    int oldest_vi = -1;
    uint32_t oldest_ts = 0;
    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active)
        {
            uint32_t ts = btn_hold_ts[voices[v].btn_id];
            if(oldest_vi < 0 || ts < oldest_ts)
            {
                oldest_ts = ts;
                oldest_vi = v;
            }
        }
    }
    return oldest_vi;
}

// Utility: among HELD buttons that are currently UNASSIGNED, return the one with OLDEST hold ts; else -1
static int FindOldestWaitingButton()
{
    int res = -1; uint32_t oldest_ts = 0;
    for(int b = 0; b < kNumKeys; ++b)
    {
        if(btn_held[b] && btn_voice[b] < 0)
        {
            if(res < 0 || btn_hold_ts[b] < oldest_ts)
            {
                oldest_ts = btn_hold_ts[b];
                res = b;
            }
        }
    }
    return res;
}

static void AssignVoiceToButton(int voice_idx, int btn_idx)
{
    voices[voice_idx].active    = true;
    voices[voice_idx].btn_id    = btn_idx;
    voices[voice_idx].timestamp = ++global_press_counter; // used only for debugging/telemetry
    btn_voice[btn_idx]          = voice_idx;
}

static void ReleaseVoice(int voice_idx)
{
    if(voices[voice_idx].active)
    {
        int b = voices[voice_idx].btn_id;
        if(b >= 0 && b < kNumKeys && btn_voice[b] == voice_idx)
            btn_voice[b] = -1;
        voices[voice_idx].active = false;
        voices[voice_idx].btn_id = -1;
    }
}

// Called on **press edge** of a button
static void OnButtonPressed(int b)
{
    btn_held[b] = true;
    btn_hold_ts[b] = ++global_press_counter; // preserve oldest-first policy

    int free_v = FindFreeVoice();
    if(free_v >= 0)
    {
        AssignVoiceToButton(free_v, b);
        return;
    }

    // No free voice: steal from OLDEST active button
    int steal_vi = FindOldestActiveVoice();
    if(steal_vi >= 0)
    {
        int victim_btn = voices[steal_vi].btn_id;
        // Mark victim as waiting (still held, no voice now)
        if(victim_btn >= 0)
            btn_voice[victim_btn] = -1;
        // Give voice to the newly pressed button
        AssignVoiceToButton(steal_vi, b);
    }
}

// Called on **release edge** of a button
static void OnButtonReleased(int b)
{
    btn_held[b] = false;

    // If this button owned a voice, free it first
    int owned_v = btn_voice[b];
    if(owned_v >= 0)
        ReleaseVoice(owned_v);

    // After freeing, if any *waiting* (held but unassigned) buttons exist, 
    // return the voice to the OLDEST waiting one (this ensures restitution to stolen owner).
    while(true)
    {
        int waiting_btn = FindOldestWaitingButton();
        if(waiting_btn < 0) break; // none waiting
        int v = FindFreeVoice();
        if(v < 0) break;            // should not happen, but be safe
        AssignVoiceToButton(v, waiting_btn);
        // assign only one per release; others will pick up on later releases
        break;
    }
}

// ===== Audio Callback =====
static void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    // pots
    volume1 = hw.adc.GetFloat(0);
    pulseW1 = hw.adc.GetFloat(1);
    volume2 = hw.adc.GetFloat(3);
    pulseW2 = hw.adc.GetFloat(4);
    detune  = hw.adc.GetFloat(5);

    // detune: -50..+50 cents
    const float cents = (detune - 0.5f) * 100.0f;
    const float detuneFactor = powf(2.0f, cents / 1200.0f);

    // configure per active voice
    for(int v = 0; v < kNumVoices; ++v)
    {
        if(voices[v].active)
        {
            const float f = KeyNoteFreq(voices[v].btn_id);
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
        // simple headroom
        mix *= 0.5f;
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
    //daisy::Pin kButtonPins[kNumKeys] = {D9, D10, D11, D12, D13, D14};
    for(int i = 0; i < kNumKeys; ++i)
    {
        keybutton[i].Init(kButtonPins[i], GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
        btn_voice[i]   = -1;
        btn_hold_ts[i] = 0;
    }

    // ADC: **configure ONLY the channels we actually read** and pass the correct count
    adc_cfg[0].InitSingle(A0); // OSC1 Volume
    adc_cfg[1].InitSingle(A1); // OSC1 Pulse Width
    // adc_cfg[2] intentionally unused
    adc_cfg[3].InitSingle(A3); // OSC2 Volume
    adc_cfg[4].InitSingle(A4); // OSC2 Pulse Width
    adc_cfg[5].InitSingle(A5); // OSC2 Detune
    hw.adc.Init(adc_cfg, 6);   // IMPORTANT: pass the real length of adc_cfg array
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

    // Poll buttons (simple edge detect); add hardware debouncing if needed
    bool prev[kNumKeys] = {false};

    while(true)
    {
        for(int b = 0; b < kNumKeys; ++b)
        {
            bool pressed = !keybutton[b].Read(); // active low with PULLUP
            if(pressed && !prev[b])
                OnButtonPressed(b);
            else if(!pressed && prev[b])
                OnButtonReleased(b);
            prev[b] = pressed;
        }

        System::Delay(1); // short poll
    }
}
