#include "daisysp.h"
#include "daisy_seed.h"
#include "hid/midi.h"

using namespace daisy;
using namespace daisysp;
using namespace seed;

DaisySeed hw;
Oscillator osc1, osc2;
AdcChannelConfig adcConfig[7];  // 7 controls (added key control)

float volume1 = 0.f, volume2 = 0.f;
float pitch1 = 0.f, pitch2 = 0.f;
float pulseW1 = 0.f, pulseW2 = 0.f;
float keyPot = 0.f;  // Key control potentiometer
int currentWaveform1 = 0, currentWaveform2 = 0;
bool lastButtonState1 = false, lastButtonState2 = false;
bool lastButtonStateQuant = false;
bool lastButtonStateScaleLock = false;

// MIDI input state
MidiUartHandler midi;
int currentMidiNote = -1;      // -1 if no note
float midiVelocity = 0.0f;     // 0.0 to 1.0


// Quantization modes
enum QuantMode { OFF, CHROMATIC, MAJOR, MINOR };
QuantMode quantizeMode = OFF;

// Scale lock mode - when true, both oscillators use the same scale notes
bool scaleLockEnabled = false;

// Convert pitch value to quantized frequency
float QuantizePitch(float pitch, QuantMode mode, int root) {
    // Define chromatic scale range (MIDI notes 24-108 = C1-C8)
    const float minNote = 24.0f;
    const float maxNote = 108.0f;
    const float range = maxNote - minNote;
    
    // Calculate MIDI note number
    float midiNote = minNote + (pitch * range);
    
    if (mode == CHROMATIC) {
        // Quantize to nearest semitone
        midiNote = roundf(midiNote);
    }
    else if (mode == MAJOR || mode == MINOR) {
        // Define scale patterns
        const int majorScale[] = {0, 2, 4, 5, 7, 9, 11}; // Major scale intervals
        const int minorScale[] = {0, 2, 3, 5, 7, 8, 10}; // Minor scale intervals
        
        const int* scale = (mode == MAJOR) ? majorScale : minorScale;
        const int numNotes = 7;
        
        // Calculate base octave and note within octave
        int octave = static_cast<int>(midiNote) / 12;
        float noteInOctave = midiNote - (octave * 12.0f);
        
        // Find closest note in scale pattern
        float minDistance = 12.0f;
        int closestScaleNote = 0;
        
        for (int i = 0; i < numNotes; i++) {
            // Calculate scale note with root applied
            float scaleNote = (scale[i] + root) % 12;
            
            // Calculate distance to scale note
            float distance = fabs(noteInOctave - scaleNote);
            
            // Check if closer in next octave
            float nextOctaveDist = fabs(noteInOctave - (scaleNote + 12.0f));
            if (nextOctaveDist < distance) {
                distance = nextOctaveDist;
                scaleNote += 12.0f;
            }
            
            // Check if closer in previous octave
            float prevOctaveDist = fabs(noteInOctave - (scaleNote - 12.0f));
            if (prevOctaveDist < distance) {
                distance = prevOctaveDist;
                scaleNote -= 12.0f;
            }
            
            if (distance < minDistance) {
                minDistance = distance;
                closestScaleNote = static_cast<int>(scaleNote);
            }
        }
        
        // Calculate final quantized MIDI note
        midiNote = octave * 12 + closestScaleNote;
    }
    
    // Convert MIDI note to frequency (A4 = 440Hz)
    return 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);
}

// MIDI event handling
void HandleMidiMessage(MidiEvent m) {
    if (m.type == NoteOn && m.data[1] > 0) {
        currentMidiNote = m.data[0];
        midiVelocity = m.data[1] / 127.0f;
        float freq = mtof(currentMidiNote);
        osc1.SetFreq(freq);
        osc2.SetFreq(freq * 1.01f); // subtle detune
        osc1.SetAmp(midiVelocity);
        osc2.SetAmp(midiVelocity);
    } else if (m.type == NoteOff || (m.type == NoteOn && m.data[1] == 0)) {
        if (m.data[0] == currentMidiNote) {
            osc1.SetAmp(0.0f);
            osc2.SetAmp(0.0f);
            currentMidiNote = -1;
            midiVelocity = 0.0f;
        }
    }
}

void AudioCallback(AudioHandle::InputBuffer in,
                  AudioHandle::OutputBuffer out,
                  size_t size)
{
    // Read all potentiometers
    volume1 = hw.adc.GetFloat(0);  // OSC1 volume
    pitch1 = hw.adc.GetFloat(1);   // OSC1 pitch
    pulseW1 = hw.adc.GetFloat(2);  // OSC1 pulse width

    // OSC2 values inverted
    volume2 = 1.0f - hw.adc.GetFloat(3);  // OSC2 volume
    pitch2 = 1.0f - hw.adc.GetFloat(4);   // OSC2 pitch
    pulseW2 = 1.0f - hw.adc.GetFloat(5);  // OSC2 pulse width
    
    // Key control
    keyPot = hw.adc.GetFloat(6);
    int root = static_cast<int>(keyPot * 11.99f);  // 0-11 (C to B)

    // Configure oscillator frequencies
    float freq1, freq2, v1, v2;
    // MIDI override: if MIDI note is held, use that
    if (currentMidiNote >= 0) {
        freq1 = mtof(currentMidiNote);
        freq2 = freq1 * 1.01f; // subtle detune
        v1 = midiVelocity;
        v2 = midiVelocity;
    } else {
        if (quantizeMode == OFF) {
            freq1 = 50.f + (pitch1 * 1950.f);
            freq2 = 50.f + (pitch2 * 1950.f);
        } else {
            // When scale lock is enabled, both oscillators use OSC1's pitch position
            // but maintain their relative offsets
            if (scaleLockEnabled && quantizeMode != CHROMATIC) {
                // Calculate base pitch position
                float basePitch = (pitch1 + pitch2) / 2.0f;
                
                // Apply OSC1 and OSC2 as offsets from the base
                freq1 = QuantizePitch(basePitch + (pitch1 - 0.5f) * 0.1f, quantizeMode, root);
                freq2 = QuantizePitch(basePitch + (pitch2 - 0.5f) * 0.1f, quantizeMode, root);
            } else {
                // Standard independent quantization
                freq1 = QuantizePitch(pitch1, quantizeMode, root);
                freq2 = QuantizePitch(pitch2, quantizeMode, root);
            }
        }
        v1 = volume1;
        v2 = volume2;   
    }
    
    osc1.SetFreq(freq1);
    osc1.SetAmp(v1);
    osc1.SetPw(pulseW1);

    osc2.SetFreq(freq2);
    osc2.SetAmp(v2);
    osc2.SetPw(pulseW2);

    for(size_t i = 0; i < size; i++)
    {
        float sig1 = osc1.Process();
        float sig2 = osc2.Process();
        out[0][i] = sig1 + sig2;
        out[1][i] = sig1 + sig2;
    }
}

void UpdateWaveform1()
{
    currentWaveform1 = (currentWaveform1 + 1) % 3;
    switch(currentWaveform1)
    {
        case 0: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

void UpdateWaveform2()
{
    currentWaveform2 = (currentWaveform2 + 1) % 3;
    switch(currentWaveform2)
    {
        case 0: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE); break;
        case 1: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW); break;
        case 2: osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI); break;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);

    // Initialize oscillators
    osc1.Init(hw.AudioSampleRate());
    osc2.Init(hw.AudioSampleRate());
    osc1.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
    osc2.SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);

    // Initialize buttons
    GPIO button1, button2, buttonQuant, buttonScaleLock;
    button1.Init(D14, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC1 waveform
    button2.Init(D13, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);  // OSC2 waveform
    buttonQuant.Init(D12, GPIO::Mode::INPUT, GPIO::Pull::PULLUP); // Quantization mode
    buttonScaleLock.Init(D11, GPIO::Mode::INPUT, GPIO::Pull::PULLUP); // Scale lock toggle

    // Configure ADC (added A6 for key control)
    adcConfig[0].InitSingle(A0);  // OSC1 Volume
    adcConfig[1].InitSingle(A1);  // OSC1 Pitch
    adcConfig[2].InitSingle(A2);  // OSC1 PWM
    adcConfig[3].InitSingle(A3);  // OSC2 Volume
    adcConfig[4].InitSingle(A4);  // OSC2 Pitch
    adcConfig[5].InitSingle(A5);  // OSC2 PWM
    adcConfig[6].InitSingle(A6);  // Key/Root control
    hw.adc.Init(adcConfig, 7);    // 7 channels now
    hw.adc.Start();

    // Initialize MIDI
    MidiUartHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;  // Usually D30 RX
    midi_cfg.transport_config.rx = D30;
    midi_cfg.transport_config.tx = D29; // Not required for input but included for completeness
    // midi_cfg.transport_config.baudrate = 31250; // Not needed, default is 31250 for MIDI
    midi.Init(midi_cfg);

    hw.StartAudio(AudioCallback);


    while(1)
    {
        // Handle OSC1 button (D14)
        bool currentButtonState1 = !button1.Read();
        if(currentButtonState1 && !lastButtonState1) {
            UpdateWaveform1();
        }
        lastButtonState1 = currentButtonState1;
        
        // Handle OSC2 button (D13)
        bool currentButtonState2 = !button2.Read();
        if(currentButtonState2 && !lastButtonState2) {
            UpdateWaveform2();
        }
        lastButtonState2 = currentButtonState2;
        
        // Handle quantization mode button (D12)
        bool currentButtonStateQuant = !buttonQuant.Read();
        if(currentButtonStateQuant && !lastButtonStateQuant) {
            // Cycle through quantization modes: OFF → CHROMATIC → MAJOR → MINOR → OFF...
            quantizeMode = static_cast<QuantMode>((static_cast<int>(quantizeMode) + 1) % 4);
        }
        lastButtonStateQuant = currentButtonStateQuant;
        
        // Handle scale lock button (D11)
        bool currentButtonStateScaleLock = !buttonScaleLock.Read();
        if(currentButtonStateScaleLock && !lastButtonStateScaleLock) {
            scaleLockEnabled = !scaleLockEnabled;
        }
        lastButtonStateScaleLock = currentButtonStateScaleLock;
        
        System::Delay(10); // Prevent busy loop, adjust as needed
    }
}