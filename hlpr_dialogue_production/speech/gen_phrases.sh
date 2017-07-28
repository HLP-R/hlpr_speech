#!/bin/sh

mkdir audio_data
rosrun cordial_tts gen_phrases.py -o phrases.yaml -d ./audio_data -v Kendra script.txt 
oggdec audio_data/*.ogg
