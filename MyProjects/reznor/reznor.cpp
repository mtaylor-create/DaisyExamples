#include "daisysp.h"
#include "daisy_pod.h"
#include "dev/mcp23x17.h"

using namespace daisysp;
using namespace daisy;

static DaisyPod   pod;
static Oscillator osc, lfo;
static MoogLadder flt;
static AdEnv      ad;
static Parameter  pitchParam, cutoffParam, lfoParam, 
                  drywetParam, crushrateParam;

static ReverbSc                                  rev;
static Tone                                      tone;

static Mcp23017 mcp;

int   wave, mode;
float vibrato, oscFreq, lfoFreq, lfoAmp, attack, release, cutoff;
float oldk1, oldk2, k1, k2;
bool  selfCycle;
float drywet = 0;

int   crushmod, crushcount;

float crushsl, crushsr;

int   panelInputA;
int   lastDigit;
int   lastUpdate;

// struct Config
//     {
//         I2CHandle::Config i2c_config;
//         uint8_t           i2c_address;
//         void              Defaults()
//         {
//             i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_1;
//             i2c_config.speed          = I2CHandle::Config::Speed::I2C_1MHZ;
//             i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
//             i2c_config.pin_config.scl = {DSY_GPIOB, 8};
//             i2c_config.pin_config.sda = {DSY_GPIOB, 9};
//             i2c_address               = 0x27;
//         }
//     };

void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update);

void Controls();

void GetReverbSample(float &outl, float &outr, float inl, float inr);

void GetCrushSample(float &outl, float &outr, float inl, float inr);

void NextSamples(float &sig)
{
    float ad_out = ad.Process();
    vibrato      = lfo.Process();

    osc.SetFreq(oscFreq + vibrato);

    sig = osc.Process();
    sig = flt.Process(sig);
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

        // left out
        //out[i] = sig;

        // right out
        //out[i + 1] = sig;
        if(mode==3||mode==4){
            GetCrushSample(sig, sig, sig, sig);
        }
        GetReverbSample(out[i], out[i+1], sig, sig);
    }
}

int getPanel(Mcp23017 mcp)
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

int getSingleDigit(Mcp23017 mcp)
{
    uint16_t mcpOutput = mcp.Read();
    int i = 0;
    while (mcpOutput > 0) {
        mcpOutput = mcpOutput >> 1;
        i++;
    }
    return (i+1)%10;
}

int main(void)
{
    // Set global variables
    float sample_rate;
    mode    = 0;
    vibrato = 0.0f;
    oscFreq = 1000.0f;
    oldk1 = oldk2 = 0;
    k1 = k2   = 0;
    attack    = .01f;
    release   = .2f;
    cutoff    = 10000;
    lfoAmp    = 1.0f;
    lfoFreq   = 0.1f;
    selfCycle = false;




    //Init everything
    pod.Init();
    pod.SetAudioBlockSize(4);
    sample_rate = pod.AudioSampleRate();
    osc.Init(sample_rate);
    flt.Init(sample_rate);
    ad.Init(sample_rate);
    lfo.Init(sample_rate);

    rev.Init(sample_rate);

    tone.Init(sample_rate);

    //Set filter parameters
    flt.SetFreq(10000);
    flt.SetRes(0.8);

    // Set parameters for oscillator
    osc.SetWaveform(osc.WAVE_SAW);
    wave = osc.WAVE_SAW;
    osc.SetFreq(440);
    osc.SetAmp(1);

    // Set parameters for lfo
    lfo.SetWaveform(osc.WAVE_SIN);
    lfo.SetFreq(0.1);
    lfo.SetAmp(1);

    //Set envelope parameters
    ad.SetTime(ADENV_SEG_ATTACK, 0.01);
    ad.SetTime(ADENV_SEG_DECAY, .2);
    ad.SetMax(1);
    ad.SetMin(0);
    ad.SetCurve(0.5);

    //set parameter parameters
    cutoffParam.Init(pod.knob1, 100, 20000, cutoffParam.LOGARITHMIC);
    pitchParam.Init(pod.knob2, 50, 5000, pitchParam.LOGARITHMIC);
    lfoParam.Init(pod.knob1, 0.25, 1000, lfoParam.LOGARITHMIC);
    drywetParam.Init(pod.knob1, 0, 1, drywetParam.LINEAR);
    crushrateParam.Init(pod.knob2, 1, 50, crushrateParam.LOGARITHMIC);

    //reverb parameters
    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    Mcp23017::Config mcpt;
    mcpt.transport_config.Defaults();
    mcpt.transport_config.i2c_address = 0b100011;
    mcp.Init(mcpt);

    //mcp.Init();
    mcp.PortMode(MCPPort::A, 0xFF, 0xFF, 0xFF);
    mcp.PortMode(MCPPort::B, 0xFF, 0xFF, 0xFF);
    //while(1) {
    panelInputA = getPanel(mcp);
    //selfCycle = true;
    //}

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while(1) {
    //panelInputA = getPanel(mcp);
    lastDigit = getSingleDigit(mcp);
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
void UpdateEncoder()
{
    wave += pod.encoder.RisingEdge();
    wave %= osc.WAVE_POLYBLEP_TRI;

    //skip ramp since it sounds like saw
    if(wave == 3)
    {
        wave = 4;
    }

    osc.SetWaveform(wave);

    mode += pod.encoder.Increment();
    mode = (mode % 5 + 5) % 5;
}

void UpdateKnobs()
{
    k1 = pod.knob1.Process();
    k2 = pod.knob2.Process();

    switch(mode)
    {
        case 0:
            //ConditionalParameter(oldk1, k1, cutoff, cutoffParam.Process());
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
        case 3:
            ConditionalParameter(oldk1, k1, cutoff, cutoffParam.Process());
            tone.SetFreq(cutoff);
            crushmod = (int)crushrateParam.Process();
        case 4:
            ConditionalParameter(oldk1, k1, drywet, drywetParam.Process());
            rev.SetFeedback(k2);
        default: break;
    }
}

void UpdateLeds()
{
    pod.led1.Set(mode == 2 || mode == 3, mode == 1 || mode == 3 || mode == 4, mode == 0 || mode == 4);
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
}

void Controls()
{
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    UpdateEncoder();

    UpdateKnobs();

    UpdateLeds();

    UpdateButtons();

    //panelInputA = getPanel(mcp);
    cutoff = panelInputA;
}

void GetCrushSample(float &outl, float &outr, float inl, float inr)
{
    crushcount++;
    crushcount %= crushmod;
    if(crushcount == 0)
    {
        crushsr = inr;
        crushsl = inl;
    }
    outl = tone.Process(crushsl);
    outr = tone.Process(crushsr);
}

void GetReverbSample(float &outl, float &outr, float inl, float inr)
{
    rev.Process(inl, inr, &outl, &outr);
    outl = drywet * outl + (1 - drywet) * inl;
    outr = drywet * outr + (1 - drywet) * inr;
}