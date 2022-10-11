#include "daisy_patch_sm.h"
#include "daisysp.h"
#include <string>


using namespace daisy;
using namespace daisysp;
using namespace patch_sm;



DaisyPatchSM patch;
ReverbSc     reverb;
Switch button;
Oscillator osc;

Chorus                chorus;

// bool state = false;
int state = 0;
int numStates = 3;

// struct lfoStruct
// {
//     Oscillator osc;
//     Parameter  freqCtrl;
//     Parameter  ampCtrl;
//     float      amp;
//     float      freq;
//     int        waveform;
//     float      value;

//     void Init(float samplerate, AnalogControl freqKnob, AnalogControl ampKnob)
//     {
//         osc.Init(samplerate);
//         osc.SetAmp(1);
//         waveform = 0;
//         freqCtrl.Init(freqKnob, .1, 35, Parameter::LOGARITHMIC);
//         ampCtrl.Init(ampKnob, 0, 1, Parameter::LINEAR);
//     }

//     void Process(DacHandle::Channel chn)
//     {
//         //read the knobs and set params
//         osc.SetFreq(freqCtrl.Process());
//         osc.SetWaveform(waveform);

//         //write to the DAC
//         patch.seed.dac.WriteValue(
//             chn,
//             uint16_t((osc.Process() + 1.f) * .5f * ampCtrl.Process() * 4095.f));
//     }
// };

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    patch.ProcessAnalogControls();


    if (state==1) {
        /** Update Params with the four knobs */
        float time_knob = patch.GetAdcValue(CV_1);
        float time      = fmap(time_knob, 0.3f, 0.99f);

        float damp_knob = patch.GetAdcValue(CV_2);
        float damp      = fmap(damp_knob, 1000.f, 19000.f, Mapping::LOG);

        float in_level = patch.GetAdcValue(CV_3);

        float send_level = patch.GetAdcValue(CV_4);


        reverb.SetFeedback(time);
        reverb.SetLpFreq(damp);

        for(size_t i = 0; i < size; i++)
        {
            float dryl  = IN_L[i] * in_level;
            float dryr  = IN_R[i] * in_level;
            float sendl = IN_L[i] * send_level;
            float sendr = IN_R[i] * send_level;
            float wetl, wetr;
            reverb.Process(sendl, sendr, &wetl, &wetr);
            OUT_L[i] = dryl + wetl;
            OUT_R[i] = dryr + wetr;
        }
    }
    else if (state == 2) {
        /** Update Params with the four knobs */
        float lfoFreq_knob = patch.GetAdcValue(CV_1);
        float lfoFreq      = fmap(lfoFreq_knob, 0.1f, 6.0f, Mapping::LOG);
        chorus.SetLfoFreq(lfoFreq*1.5, lfoFreq);

        float delay_knob = patch.GetAdcValue(CV_2);
        float dVal = fmap(delay_knob, 0.f, 1.f);

        chorus.SetDelay(dVal*0.8, dVal);



        float fb_knob = patch.GetAdcValue(CV_3);
        float fb      = fmap(fb_knob, 0.f, 1.f);
        chorus.SetFeedback(fb);

        float dry_level = patch.GetAdcValue(CV_4);

        for(size_t i = 0; i < size; i++)
        {
            float chorusIn = (IN_L[i] + IN_R[i])/2;
            chorus.Process(chorusIn);
            out[0][i] = chorus.GetLeft() + IN_L[i] * dry_level;
            out[1][i] = chorus.GetRight() + IN_R[i] * dry_level;
        }
    }
    else {
        float coarse_knob = patch.GetAdcValue(CV_1);
        float coarse      = fmap(coarse_knob, 0.f, 10.f);

        float amp_knob = patch.GetAdcValue(CV_2);
        float amp       = fmap(amp_knob, 0.05f, 5.f);

        osc.SetFreq(coarse);
        osc.SetAmp(amp);
        float sig;

        //for(size_t i = 0; i < size; i++)
        //{
        //    sig = osc.Process();
        //}
        patch.WriteCvOut(CV_OUT_2, 2.5+osc.Process());

    }
}

//lfoStruct lfos[2];

int main(void)
{
    patch.Init();
    reverb.Init(patch.AudioSampleRate());
    button.Init(patch.B8);
    osc.Init(patch.AudioSampleRate());

    chorus.Init(patch.AudioSampleRate());
    chorus.SetLfoFreq(.33f, .2f);
    chorus.SetLfoDepth(1.f, 1.f);
    chorus.SetDelay(.75f, .9f);

    //init the lfos
    //lfos[0].Init(samplerate, patch.controls[0], patch.controls[1]);
    //lfos[1].Init(samplerate, patch.controls[2], patch.controls[3]);

    patch.StartAudio(AudioCallback);
    int lastButton = false;
    while(1) {
        patch.Delay(10);
        button.Debounce();
        if ((not lastButton) and button.Pressed()) {
            lastButton = true;
            state = (state + 1) % numStates;
        }
        lastButton = button.Pressed();

        /** Set the onboard led to the current state */
        if (state == 0) {
            dsy_gpio_write(&patch.gate_out_1, false);
            dsy_gpio_write(&patch.gate_out_2, false);
        }
        else if (state == 1) {
            dsy_gpio_write(&patch.gate_out_1, false);
            dsy_gpio_write(&patch.gate_out_2, true);
        }
        else if (state == 2) {
            dsy_gpio_write(&patch.gate_out_1, true);
            dsy_gpio_write(&patch.gate_out_2, false);
        }
        else if (state == 3) {
            dsy_gpio_write(&patch.gate_out_1, true);
            dsy_gpio_write(&patch.gate_out_2, true);
        }

        //float voltage = patch.GetRandomFloat(0.0, 5.0);
        //patch.WriteCvOut(CV_OUT_1, voltage);


    }
}
