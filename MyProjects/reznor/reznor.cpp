#include "daisysp.h"
#include "daisy_seed.h"
#include "dev/mcp23x17.h"

using namespace daisysp;
using namespace daisy;
using namespace daisy::seed;

// Daisy objects
static DaisySeed hardware;
static Oscillator osc, osc1, osc2, lfo;
static MoogLadder flt;
static AdEnv      ad;
static Parameter  pitchParam, cutoffParam, crushCutoffParam, lfoParam, 
                  drywetParam, crushrateParam, detuneParam;
static ReverbSc                                  rev;
static Tone                                      tone;
ClockedNoise noise;
FractalRandomGenerator<ClockedNoise, 5> fract;

// Controls
static Mcp23017 panelA[2];
static Mcp23017 panelB[2];
static Mcp23017 mcpButtons[2];
float analogKnobA, analogKnobB, analogKnobC, analogKnobD, analogKnobE;
float analogPanelA, analogPanelB, analogPanelC;

// Outputs
GPIO LedA, LedB, LedC, LedD;

// Globals
int   wave, mode;
float vibrato, oscFreq, lfoFreq, lfoAmp, attack, release, cutoff, crushCutoff;
float detune;
float revFeedback;
float oldk1, oldk2, k1, k2;
bool  selfCycle;
float drywet = 0;
int   crushcount = 0;
float crushmod = 1;
float crushedSig;
float crushsl, crushsr;

int   panelInputA, panelInputB;
int   mcpButtonState;

void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update);

void Controls();

void UpdateMeters();

void GetReverbSample(float &outl, float &outr, float inl, float inr);

float GetCrushSample(float sig);

//https://hackernoon.com/bit-manipulation-in-c-and-c-1cs2bux
bool get_bit(int num, int position) 
{
	bool bit = num & (1 << position);
	return bit;
}

void NextSamples(float &sig)
{
    float ad_out = ad.Process();
    vibrato      = lfo.Process();

    osc.SetFreq(oscFreq + vibrato);
    osc1.SetFreq(oscFreq + vibrato + detune);
    osc2.SetFreq(oscFreq + vibrato - detune);



    sig = (osc.Process() + osc1.Process() + osc2.Process())/3;
    sig = flt.Process(sig);

    if(wave == 3)
    {
        fract.SetFreq(fabsf(oscFreq));
        fract.SetColor(fabsf(cutoff/20000));
        sig = fract.Process();
    }
    
    sig *= ad_out;
}

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    Controls();

    for(size_t i = 0; i < size; i += 2)
    {
        float sig;
        NextSamples(sig);

        //sig = GetCrushSample(sig);
        GetReverbSample(out[i], out[i+1], sig, sig);
    }
} 

int getMcpButtons(Mcp23017 mcp0)
{
    uint16_t mcpOutput = mcp0.Read();
    return mcpOutput + 0;
}

void configPanel(Mcp23017 mcp[2], int addr0, int addr1)
{
    Mcp23017::Config mcpt0;
    mcpt0.transport_config.Defaults();
    mcpt0.transport_config.i2c_address = addr0;//0b100111;
    mcp[0].Init(mcpt0);
    mcp[0].PortMode(MCPPort::A, 0xFF, 0xFF, 0xFF);
    mcp[0].PortMode(MCPPort::B, 0xFF, 0xFF, 0xFF);

    Mcp23017::Config mcpt1;
    mcpt1.transport_config.Defaults();
    mcpt1.transport_config.i2c_address = addr1;//0b100011;
    mcp[1].Init(mcpt1);
    mcp[1].PortMode(MCPPort::A, 0xFF, 0xFF, 0xFF);
    mcp[1].PortMode(MCPPort::B, 0xFF, 0xFF, 0xFF);
}

int getPanelLSDs(Mcp23017 mcp)
{
    uint16_t mcpOutput = mcp.Read();
    uint8_t dig0 = mcpOutput % 0b10000;
    dig0 = (dig0 > 7) ? dig0 - 6: dig0;
    uint8_t dig1 = (mcpOutput >> 4) % 0b10000;
    dig1 = (dig1 > 7) ? dig1 - 6: dig1;
    uint8_t dig2 = (mcpOutput >> 8) % 0b10000;
    dig2 = (dig2 > 7) ? dig2 - 6: dig2;
    uint8_t dig3 = (mcpOutput >> 12) % 0b10000;
    dig3 = (dig3 > 7) ? dig3 - 6: dig3;
    return dig0 + 10*dig1 + 100*dig2 + 1000*dig3;
}

int getPanelMSD(Mcp23017 mcp)
{
    uint16_t mcpOutput = mcp.Read();
    int i = 0;
    while (mcpOutput > 0) {
        mcpOutput = mcpOutput >> 1;
        i++;
    }
    return (i+1)%10;
}

int getPanelDigits(Mcp23017 mcp[2])
{
    return getPanelLSDs(mcp[0]) + 10000*getPanelMSD(mcp[1]);
}

int main(void)
{
    // Set global variables
    float sample_rate;
    mode    = 0;
    vibrato = 0.0f;
    oscFreq = 440.0f;
    oldk1 = oldk2 = 0;
    k1 = k2   = 0;
    attack    = .01f;
    release   = .2f;
    cutoff    = 10000;
    lfoAmp    = .01f;
    lfoFreq   = 0.1f;
    selfCycle = true;

    // Init everything
    hardware.Init();
    AdcChannelConfig adcConfig[8];
    adcConfig[0].InitSingle(hardware.GetPin(19));
    adcConfig[1].InitSingle(hardware.GetPin(18));
    adcConfig[2].InitSingle(hardware.GetPin(17));
    adcConfig[3].InitSingle(hardware.GetPin(16));
    adcConfig[4].InitSingle(hardware.GetPin(15));
    adcConfig[5].InitSingle(hardware.GetPin(20));
    adcConfig[6].InitSingle(hardware.GetPin(21));
    adcConfig[7].InitSingle(hardware.GetPin(24));
    hardware.adc.Init(adcConfig, 8);
    hardware.adc.Start();

    LedA.Init(D7, GPIO::Mode::OUTPUT);
    LedB.Init(D8, GPIO::Mode::OUTPUT);
    LedC.Init(D9, GPIO::Mode::OUTPUT);
    LedD.Init(D10, GPIO::Mode::OUTPUT);

    hardware.SetAudioBlockSize(4);
    sample_rate = hardware.AudioSampleRate();
    osc.Init(sample_rate);
    osc1.Init(sample_rate);
    osc2.Init(sample_rate);
    flt.Init(sample_rate);
    ad.Init(sample_rate);
    lfo.Init(sample_rate);
    rev.Init(sample_rate);
    tone.Init(sample_rate);

    // Filter params
    flt.SetFreq(15000);
    flt.SetRes(0);

    // Osc params
    osc.SetWaveform(osc.WAVE_SAW);
    osc1.SetWaveform(osc.WAVE_SAW);
    osc2.SetWaveform(osc.WAVE_SAW);

    wave = osc.WAVE_SAW;
    osc.SetFreq(440);
    osc.SetAmp(1);
    osc1.SetFreq(440);
    osc1.SetAmp(1);
    osc2.SetFreq(440);
    osc2.SetAmp(1);

    // LFO params
    lfo.SetWaveform(osc.WAVE_SIN);
    lfo.SetFreq(0.1);
    lfo.SetAmp(1);

    // Envelope params
    ad.SetTime(ADENV_SEG_ATTACK, 0.01);
    ad.SetTime(ADENV_SEG_DECAY, 1);
    ad.SetMax(1);
    ad.SetMin(0);
    ad.SetCurve(0.5);

    // Noise params
    fract.Init(sample_rate);
    fract.SetFreq(sample_rate / 10.f);

    //set parameter parameters
    /* cutoffParam.Init(pod.knob1, 100, 20000, cutoffParam.LOGARITHMIC);
    crushCutoffParam.Init(pod.knob1, 600, 30000, crushCutoffParam.LOGARITHMIC);
    pitchParam.Init(pod.knob2, 50, 5000, pitchParam.LOGARITHMIC);
    lfoParam.Init(pod.knob1, 0.25, 1000, lfoParam.LOGARITHMIC);
    drywetParam.Init(pod.knob1, 0, 1, drywetParam.LINEAR);
    crushrateParam.Init(pod.knob2, 0.9, 100, crushrateParam.LOGARITHMIC);
    detuneParam.Init(pod.knob2, 0, 10, detuneParam.LINEAR); */

    // Crush params
    crushCutoff = 30000;
    tone.SetFreq(crushCutoff);

    // Reverb parameters
    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    // Config i2c
    configPanel(panelA, 0b100110, 0b100010);  
    configPanel(panelB, 0b100111, 0b100011);  
    configPanel(mcpButtons, 0b100000, 0b100000);  

    // Start DAC
    DacHandle::Config dacCfg;
	dacCfg.bitdepth = DacHandle::BitDepth::BITS_12;
	dacCfg.buff_state = DacHandle::BufferState::ENABLED;
	dacCfg.mode = DacHandle::Mode::POLLING;
	dacCfg.chn = DacHandle::Channel::BOTH;
	hardware.dac.Init(dacCfg);

    // Start audio callback
    hardware.StartAudio(AudioCallback);

    while(1) {
    panelInputA = getPanelDigits(panelA);
    panelInputB = getPanelDigits(panelB);
    mcpButtonState = getMcpButtons(mcpButtons[0]);

    LedA.Write(get_bit(mcpButtonState, 0));
    LedB.Write(get_bit(mcpButtonState, 1));
    LedC.Write(get_bit(mcpButtonState, 2));
    LedD.Write(get_bit(mcpButtonState, 3));

    UpdateMeters();

    if (!ad.IsRunning()) {
        ad.Trigger();
    }

    }
}

//Updates values if knob had changed
void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update)
{
    if(abs(oldVal - newVal) > 0.00005)
    {
        param = update;
    }
}


//Controls Helpers
/* void UpdateEncoder()
{
    wave += pod.encoder.RisingEdge();
    wave %= osc.WAVE_POLYBLEP_TRI;

    //skip ramp since it sounds like saw
    //if(wave == 3)
    //{
    //    wave = 4;
    //}

    osc.SetWaveform(wave);
    osc1.SetWaveform(wave);
    osc2.SetWaveform(wave);


    mode += pod.encoder.Increment();
    mode = (mode % 6 + 6) % 6;
}

void UpdateKnobs()
{
    k1 = pod.knob1.Process();
    k2 = pod.knob2.Process();

    switch(mode)
    {
        case 0:
            ConditionalParameter(oldk1, k1, cutoff, cutoffParam.Process());
            ConditionalParameter(oldk2, k2, oscFreq, pitchParam.Process());
            flt.SetFreq(cutoff);
            break;
        case 1:
            ConditionalParameter(oldk1, k1, attack, pod.knob1.Process());
            ConditionalParameter(oldk2, k2, release, pod.knob2.Process());
            ad.SetTime(ADENV_SEG_ATTACK, attack);
            ad.SetTime(ADENV_SEG_DECAY, release);
            break;
        case 2:
            ConditionalParameter(oldk1, k1, lfoFreq, lfoParam.Process());
            ConditionalParameter(oldk2, k2, lfoAmp, pod.knob2.Process());
            lfo.SetFreq(lfoFreq);
            lfo.SetAmp(lfoAmp * 100);
            break;
        case 3:
            ConditionalParameter(oldk1, k1, crushCutoff, crushCutoffParam.Process());
            ConditionalParameter(oldk2, k2, crushmod, crushrateParam.Process());
            tone.SetFreq(crushCutoff);
            //crushmod = (int)crushrateParam.Process();
            break;
        case 4:
            ConditionalParameter(oldk1, k1, drywet, drywetParam.Process());
            ConditionalParameter(oldk2, k2, revFeedback, pod.knob2.Process());
            rev.SetFeedback(revFeedback);
            break;
        case 5:
            ConditionalParameter(oldk2, k2, detune, detuneParam.Process());

        default: break;
    }
}

void UpdateLeds()
{
    pod.led1.Set(mode == 2 || mode == 3 || mode == 5, 
                 mode == 1 || mode == 3 || mode == 4, 
                 mode == 0 || mode == 4 || mode == 5);
    pod.led2.Set(0, selfCycle, selfCycle);

    oldk1 = k1;
    oldk2 = k2;

    pod.UpdateLeds();
}

void UpdateButtons()
{
    if(pod.button1.RisingEdge() || (selfCycle && !ad.IsRunning()))
    {
        ad.Trigger();
    }

    if(pod.button2.RisingEdge())
    {
        selfCycle = !selfCycle;
    }
}*/

void UpdatePanels()
{
    
}

void UpdateButtons()
{
    
}

void UpdateKnobs()
{
    analogKnobA = hardware.adc.GetFloat(0);
    analogKnobB = hardware.adc.GetFloat(1);
    analogKnobC = hardware.adc.GetFloat(2);
    analogKnobD = hardware.adc.GetFloat(3);
    analogKnobE = hardware.adc.GetFloat(4);
    analogPanelA = hardware.adc.GetFloat(5);
    analogPanelB = hardware.adc.GetFloat(6);
    analogPanelC = hardware.adc.GetFloat(7);

    oscFreq = analogKnobA * 2000;
    flt.SetFreq(analogKnobB * 20000);
    flt.SetRes(analogKnobC);
}

void UpdateMeters()
{
    hardware.dac.WriteValue(DacHandle::Channel::ONE, analogKnobC*650);
	hardware.dac.WriteValue(DacHandle::Channel::TWO, analogKnobB*650);
}

void UpdateLeds()
{
    LedA.Write(get_bit(mcpButtonState, 0));
    LedB.Write(get_bit(mcpButtonState, 1));
    LedC.Write(get_bit(mcpButtonState, 2));
    LedD.Write(get_bit(mcpButtonState, 3));
}

void Controls()
{
    UpdatePanels();

    UpdateButtons();

    UpdateKnobs();

    UpdateLeds();
}

/*
float GetCrushSample(float sig)
{
    if (crushmod<=1) {
        return sig;
    }
    crushcount++;
    //crushcount %= (int)crushmod;
    if(crushcount > crushmod)
    {
        crushedSig = sig;
        crushcount = 0;
    }
    crushedSig = tone.Process(crushedSig);
    return crushedSig;
    //return tone.Process(crushedSig);
} */

void GetReverbSample(float &outl, float &outr, float inl, float inr)
{
    rev.Process(inl, inr, &outl, &outr);
    outl = drywet * outl + (1 - drywet) * inl;
    outr = drywet * outr + (1 - drywet) * inr;
}