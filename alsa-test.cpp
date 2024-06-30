//This code tests the ALSA libraries by opening the audio device, reading a frame from the IP device and writing it out to the OP device. 

#include "ALSADevices.hpp"
#include "alsafunc.cpp"
#include <iostream>
#include<string>

//Constants:
#define SAMPLING_RATE 16000
#define CHANNELS 2
#define FRAMES_PER_PERIOD 64
const snd_pcm_format_t FORMAT = SND_PCM_FORMAT_S16_BE;  //Formats reference: https://vovkos.github.io/doxyrest/samples/alsa/enum_snd_pcm_format_t.html 


ALSACaptureDevice microphone("plughw:1,0,0", SAMPLING_RATE, CHANNELS, FRAMES_PER_PERIOD, FORMAT);   //Init fn for ALSA Input device 
ALSAPlaybackDevice speaker("default", SAMPLING_RATE, CHANNELS, FRAMES_PER_PERIOD, FORMAT);          //Init fn for ALSA Output device


int main() {
    microphone.open();
    speaker.open();
    char* buffer = microphone.allocate_buffer();
    unsigned int frames_captured, frames_played;

    do{
        frames_captured = microphone.capture_into_buffer(buffer, FRAMES_PER_PERIOD);
        frames_played = speaker.play_from_buffer(buffer, FRAMES_PER_PERIOD);
        std::cout << "Captured,Played ---> " << frames_captured << "," << frames_played << std::endl;
    }while(1);

    microphone.close();
    speaker.close();
    return 0;
}