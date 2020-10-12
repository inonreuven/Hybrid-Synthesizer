# Hybrid-Synthesizer
University’s Final project 

## Table of contents
* [Inspiration](#inspiration)
* [Motivation](#motivation)
* [Block Diagram](#block-diagram)
* [Clock Generator](#clock-generator)
* [Step Sequencer](#step-sequencer)
* [MIDI2CV](#midi2cv)

## Inspiration
As electronic music industry grows, and the warm analog sound returns to the forefront, the Hybrid Musical Monophonic Synthesizer offers a full analog signal path and a digital precise tempo control, periodic sequences generation and MIDI notes conversion. The synth consists two audio VCOs. The frequency of the VCOs is controlled in two ways: MIDI Keyboard or Step Sequencer. The VCOs signals are summed , shaped by a 4-poles filter and amplified.  In addition,  there are three LFOs (in order to generate periodic signals) and two EGs (to generate shaped pulses)  to control a variety of parameters of the synth.

## Motivation
The synth consists two main parts; digital and analog, when this project focuses only on the digital part. The core of my hybrid musical monophonic synthesizer is the [STM32F411RE Nucleo board](https://os.mbed.com/platforms/ST-Nucleo-F411RE/). This tutorial provide information how to design:
* Clock Generator - open the EG to produce shaped pulses and turn on the sequencer.
*	Step Sequencer - generates up to 16 different DC analog signals (triggered by the clock) which control the frequency of the oscillators to play notes.
* MIDI2CV - convert MIDI messages to control voltage signals.

## Block Diagram
<img src="Images/BlockDiagram.jpg" width="100">
![](Images/BlockDiagram.jpg)


