#include "daisysp.h"
#include "daisy_seed.h"
#include "dev/mcp23x17.h"

using namespace daisysp;
using namespace daisy;
using namespace daisy::seed;

// Daisy objects
static DaisySeed hardware;
MidiUsbHandler midi;
static Oscillator osc, osc1, osc2, lfo;
static MoogLadder fltMoog;
static Svf flt;
static AdEnv      adVol, adFilt;
static Parameter  pitchParam, cutoffParam, crushCutoffParam, lfoParam, 
                  drywetParam, crushrateParam, detuneParam;
static ReverbSc                                  rev;
static Tone                                      tone;
ClockedNoise noise;
FractalRandomGenerator<ClockedNoise, 5> fract;

Chorus                chorus;

// Controls
static Mcp23017 panelA[2];
static Mcp23017 panelB[2];
static Mcp23017 mcpButtons[2];
float analogKnobA, analogKnobB, analogKnobC, analogKnobD, analogKnobE;
float analogPanelA, analogPanelB, analogPanelC;
int   aPint_A, aPint_B, aPint_C;

bool buttonTrigger, buttonCycle, buttonRes, buttonAttVol, buttonRelVol;
bool buttonAttFilt, buttonRelFilt;
bool buttonFmode, buttonWave, buttonWave_last, buttonLfoToPitch, buttonLfoToFilt;
bool buttonDrone, buttonEffectsOn, buttonMultiOsc, buttonMoog, buttonVol;

float filterCutoff, filterModEnv, filterMax, filterMin;
float lfoOut = 0;
float gain = 1;
// Outputs
GPIO LedA, LedB, LedC, LedD;

// Globals
int   wave, mode;
float vibrato, oscFreq, lfoFreq, lfoAmp, attack, release, cutoff, crushCutoff;
float oscOffset0, oscOffset1, oscOffset2;
float detune;
float revFeedback;
float oldk1, oldk2, k1, k2;
bool  selfCycle;
float revDryWet = 0;
int   crushcount = 0;
float crushmod = 1;
float crushDryWet = 0;
float crushedSig;
float crushsl, crushsr;
float lfoPitchMod = 0;
float lfoFiltMod = 0;

float chorusDryWet = 0;

int   panelInputA, panelInputB;
int   mcpButtonState;

int   chordIntervals [] = {0, 3, 4, 6, 7, 8, 9, 10, 11, 12};

void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update);

void Controls();

void UpdateMeters();

void UpdateWave();

void UpdateIndividualButtons();

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
    float ad_out = adVol.Process();
    if (buttonDrone) {
        ad_out = 1;
    }
    lfoOut    = lfo.Process();
    vibrato = lfoOut*lfoPitchMod;

    osc.SetFreq(oscOffset0 * (1 + vibrato));
    osc1.SetFreq((oscOffset1 + detune) * (1 + vibrato));
    osc2.SetFreq((oscOffset2 - detune) * (1 + vibrato));

    float ad_out_filt = adFilt.Process();
    float sigCutoff = std::max(filterMin, filterCutoff 
                                          + (ad_out_filt * filterModEnv) 
                                          + lfoOut*lfoFiltMod);
    sigCutoff = std::min(filterMax, sigCutoff);
    flt.SetFreq(sigCutoff);
    fltMoog.SetFreq(sigCutoff);
    if (buttonMultiOsc) {
        sig = (osc.Process() + osc1.Process() + osc2.Process())/3;
    }
    else {
        sig = osc.Process();
    }

    if (buttonMoog) {
        sig = fltMoog.Process(sig);
    }
    else {
        flt.Process(sig);
        if (buttonFmode) {
            sig = flt.High();
        }
        else {
            sig = flt.Low();
        }
    }

    if(wave == 3)
    {
        fract.SetFreq(fabsf(oscOffset0));
        fract.SetColor(fabsf(sigCutoff/10000));
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
        float sigL;
        float sigR;
        NextSamples(sig);

        if (buttonEffectsOn) {
            //sig = GetCrushSample(sig);
            //sig = (crushDryWet * GetCrushSample(sig) + (1 - crushDryWet) * sig)/2;
            chorus.Process(sig);
            sigL = (chorusDryWet * chorus.GetLeft() + (1 - chorusDryWet) * sig);
            sigR = (chorusDryWet * chorus.GetRight() + (1 - chorusDryWet) * sig);
            sigL = gain * (crushDryWet * GetCrushSample(sigL) + (1 - crushDryWet) * sigL);
            sigR = gain * (crushDryWet * GetCrushSample(sigR) + (1 - crushDryWet) * sigR);
            GetReverbSample(out[i], out[i+1], sigL, sigR);
        }
        else {
            out[i] = gain * sig;
            out[i+1] = gain * sig;
        }
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

int getKthDigit(int n, int k) {
    int result = (n / pow(10, k));
    return result % 10;
}

int getAnalogPanelDigit(float fpv) {
    return ((fpv + 0.056)*9);
}

int main(void)
{
    // Set global variables
    float sample_rate;
    mode    = 0;
    vibrato = 0.0f;
    oscFreq = 440.0f;
    oscOffset1 = 0;
    oscOffset2 = 0;
    detune = 0;
    oldk1 = oldk2 = 0;
    k1 = k2   = 0;
    attack    = .01f;
    release   = .2f;
    filterCutoff = 15000;
    filterMax = 32000;
    filterMin = 10;
    cutoff    = 10000;
    lfoAmp    = .01f;
    lfoFreq   = 0.1f;
    selfCycle = false;
    filterModEnv = 0;

    // Init everything
    hardware.Init();
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    midi.Init(midi_cfg);



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
    fltMoog.Init(sample_rate);
    adVol.Init(sample_rate);
    adFilt.Init(sample_rate);
    lfo.Init(sample_rate);
    rev.Init(sample_rate);
    tone.Init(sample_rate);
    chorus.Init(sample_rate);


    // Filter params
    flt.SetFreq(filterCutoff);
    fltMoog.SetFreq(filterCutoff);
    flt.SetRes(0);
    fltMoog.SetRes(0.5);
    flt.SetDrive(0.9);

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
    adVol.SetTime(ADENV_SEG_ATTACK, 0.01);
    adVol.SetTime(ADENV_SEG_DECAY, 1);
    adVol.SetMax(1);
    adVol.SetMin(0);
    adVol.SetCurve(0.5);

    adFilt.SetTime(ADENV_SEG_ATTACK, 0.01);
    adFilt.SetTime(ADENV_SEG_DECAY, 1);
    adFilt.SetMax(1);
    adFilt.SetMin(0);
    adFilt.SetCurve(0.5);

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

    // Chorus params
    chorus.SetLfoFreq(.2f, .2f);
    chorus.SetLfoDepth(1.f, 1.f);
    chorus.SetDelay(.9f, .9f);
    chorus.SetFeedback(0);

    // Crush params
    crushCutoff = 30000;
    tone.SetFreq(crushCutoff);

    // Reverb parameters
    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    // Config i2c
    configPanel(panelB, 0b100110, 0b100010);  
    configPanel(panelA, 0b100111, 0b100011);  
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
    midi.Listen();
    while(midi.HasEvents())
    {
        /** Pull the oldest one from the list... */
        auto msg = midi.PopEvent();
        switch(msg.type)
        {
            case NoteOn:
            {
                /** and change the frequency of the oscillator */
                auto note_msg = msg.AsNoteOn();
                if(note_msg.velocity != 0)
                    oscFreq = mtof(note_msg.note);
            }
            break;
                // Since we only care about note-on messages in this example
                // we'll ignore all other message types
            default: break;
        }
    }


    panelInputB = getPanelDigits(panelB);
    panelInputA = getPanelDigits(panelA);
    mcpButtonState = getMcpButtons(mcpButtons[0]);
    UpdateIndividualButtons();

    // LedA.Write(get_bit(mcpButtonState, 0));
    // LedB.Write(get_bit(mcpButtonState, 1));
    // LedC.Write(get_bit(mcpButtonState, 2));
    // LedD.Write(get_bit(mcpButtonState, 3));

    UpdateMeters();

    if (buttonWave && !buttonWave_last) {
        UpdateWave();
    }
    buttonWave_last = buttonWave;

    if (buttonTrigger || (buttonCycle && !adVol.IsRunning())) {
    //if (!ad.IsRunning()) {
        adVol.Trigger();
        adFilt.Trigger();
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
void UpdateWave()
{
    wave += 1;
    wave %= osc.WAVE_POLYBLEP_TRI;

    //skip ramp since it sounds like saw
    //if(wave == 3)
    //{
    //    wave = 4;
    //}

    osc.SetWaveform(wave);
    osc1.SetWaveform(wave);
    osc2.SetWaveform(wave);
}

/* 
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
    //reverb
    revDryWet = getKthDigit(panelInputB, 0) / 9.0;
    revFeedback = getKthDigit(panelInputB, 1) / 9.0;
    rev.SetFeedback(revFeedback);

    //crush
    crushDryWet = getKthDigit(panelInputB, 2) / 9.0;
    crushCutoff = 500 + 2500*getKthDigit(panelInputB, 3);
    tone.SetFreq(crushCutoff);
    crushmod = pow(2, getKthDigit(panelInputB, 4));

    //chorus
    //chorus.SetLfoFreq(0.5 + 2 * getKthDigit(panelInputA, 4));
    chorus.SetLfoFreq(pow(2, getKthDigit(panelInputA, 4)) / 10.0);
    chorus.SetLfoDepth(getKthDigit(panelInputA, 3) / 9.0);
    chorus.SetDelay(getKthDigit(panelInputA, 2) / 9.0);
    chorus.SetFeedback(getKthDigit(panelInputA, 1) / 9.5);
    chorusDryWet = getKthDigit(panelInputA, 0) / 9.0;
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

    if (buttonVol) {
        oscFreq = analogKnobA * 2000;
    }
    else {
        gain = pow(2,analogKnobA * 2);
    }

    // Analog Panel
    analogPanelA = hardware.adc.GetFloat(5);
    analogPanelB = hardware.adc.GetFloat(6);
    analogPanelC = hardware.adc.GetFloat(7);
    aPint_A = getAnalogPanelDigit(analogPanelA); 
    aPint_B = getAnalogPanelDigit(analogPanelB); 
    aPint_C = getAnalogPanelDigit(analogPanelC);
    aPint_A = chordIntervals[aPint_A] + 12 * (((aPint_C + 1) / 3) % 3);
    aPint_B = chordIntervals[aPint_B] + 12 * (((aPint_C) / 3) % 3);

    oscOffset1 = oscFreq * pow(2, (aPint_A/12.0));
    oscOffset2 = oscFreq * pow(2, (aPint_B/12.0));
    oscOffset0 = oscFreq * pow(2, ((aPint_C+2)/3) % 2);




    // Vol env
    if (buttonAttVol) {
        adVol.SetTime(ADENV_SEG_ATTACK, 5*pow(analogKnobB, 2));
    }
    if (buttonRelVol) {
        adVol.SetTime(ADENV_SEG_DECAY, 5*pow(analogKnobB, 2));
    }
    if (!(buttonAttVol || buttonRelVol)) {
        detune = analogKnobB * 10;
    }

    // Filter section
    if (buttonRes) {
        flt.SetRes(analogKnobD);
        fltMoog.SetRes(analogKnobD);
    }
    else {
        filterCutoff = pow(2, 5 + analogKnobD * 10);
    }

    // Filter env
    if (buttonAttFilt) {
        adFilt.SetTime(ADENV_SEG_ATTACK, 5*pow(analogKnobE, 2));
    }
    if (buttonRelFilt) {
        adFilt.SetTime(ADENV_SEG_DECAY, 5*pow(analogKnobE, 2));
    }
    if (!(buttonAttFilt || buttonRelFilt)) {
        //filterModEnv = filterMax*(analogKnobE - 0.5)/2;
        filterModEnv = pow(40*(analogKnobE - 0.5),3);
    }

    // LFO section
    if (buttonLfoToPitch) {
        lfoPitchMod = (pow(2, analogKnobC*2)-1);
    }
    if (buttonLfoToFilt) {
        lfoFiltMod = (pow(2, analogKnobC*10)-1);
    }
    if (!(buttonLfoToPitch||buttonLfoToFilt)) {
        lfo.SetFreq(pow(10, analogKnobC*5)/20);
    }

    
}

void UpdateMeters()
{
    //hardware.dac.WriteValue(DacHandle::Channel::ONE, analogKnobC*650);
	//hardware.dac.WriteValue(DacHandle::Channel::TWO, analogKnobB*650);
	hardware.dac.WriteValue(DacHandle::Channel::ONE, (lfoOut+1) * 300);
    hardware.dac.WriteValue(DacHandle::Channel::TWO, adVol.GetValue() *650);

}

void UpdateIndividualButtons()
{
    buttonTrigger = get_bit(mcpButtonState, 0);
    buttonCycle = get_bit(mcpButtonState, 3);
    buttonRes = get_bit(mcpButtonState, 12);
    buttonAttVol = get_bit(mcpButtonState, 4);
    buttonRelVol = get_bit(mcpButtonState, 5);
    buttonAttFilt = get_bit(mcpButtonState, 9);
    buttonRelFilt = get_bit(mcpButtonState, 8);
    buttonFmode = get_bit(mcpButtonState, 13);
    buttonWave = get_bit(mcpButtonState, 1);
    buttonLfoToPitch = get_bit(mcpButtonState, 15);
    buttonLfoToFilt = get_bit(mcpButtonState, 14);
    buttonDrone = get_bit(mcpButtonState, 2);
    buttonEffectsOn = get_bit(mcpButtonState, 10);
    buttonMultiOsc = get_bit(mcpButtonState, 11);
    buttonMoog = get_bit(mcpButtonState, 6);
    buttonVol = get_bit(mcpButtonState, 7);

}


void UpdateLeds()
{
    LedA.Write(buttonTrigger);
    LedB.Write(buttonCycle);
    LedC.Write(adVol.IsRunning());
    LedD.Write(adVol.GetCurrentSegment() == 1);
}

void Controls()
{
    UpdatePanels();

    UpdateButtons();

    UpdateKnobs();

    UpdateLeds();
}


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
}

void GetReverbSample(float &outl, float &outr, float inl, float inr)
{
    rev.Process(inl, inr, &outl, &outr);
    outl = revDryWet * outl + (1 - revDryWet) * inl;
    outr = revDryWet * outr + (1 - revDryWet) * inr;
}