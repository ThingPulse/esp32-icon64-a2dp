/*
MIT License

Copyright (c) 2020 ThingPulse GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorAAC.h"
#include "AudioOutputI2S.h"
#include "BluetoothA2DPSink.h"
#include "icons.h"
#include "sounds.h"
#include <FastLED.h>
#include <arduinoFFT.h>

// Audio Settings
#define I2S_DOUT    25
#define I2S_BCLK    26
#define I2S_LRC     22
#define MODE_PIN    33

// LED Settings
#define LED_COUNT   64
#define LED_PIN     32
#define CHANNEL     0

#define DATA_PIN 32

// FFT Settings
#define NUM_BANDS 8
#define SAMPLES 512
#define SAMPLING_FREQUENCY 44100

#define BRIGHTNESS 100

#define DEVICE_NAME "ThingPulse-Icon64"

arduinoFFT FFT = arduinoFFT();
BluetoothA2DPSink a2dp_sink;
CRGB leds[LED_COUNT];

int pushButton = 39;

float amplitude = 200.0;

int32_t peak[] = {0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];

double brightness = 0.25;

QueueHandle_t queue;

int16_t sample_l_int;
int16_t sample_r_int;

int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;

static const i2s_pin_config_t pin_config = {.bck_io_num = I2S_BCLK,
                                            .ws_io_num = I2S_LRC,
                                            .data_out_num = I2S_DOUT,
                                            .data_in_num = I2S_PIN_NO_CHANGE};

uint8_t hueOffset = 0;

// audio state management
bool devicePlayedAudio = false;
esp_a2d_audio_state_t currentAudioState = ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND;

// device connection management
bool bleDeviceConnected = false;

uint8_t getLedIndex(uint8_t x, uint8_t y) {
  // x = 7 - x;
  if (y % 2 == 0) {
    return y * 8 + x;
  } else {
    return y * 8 + (7 - x);
  }
}

void createBands(int i, int dsize) {
  uint8_t band = 0;
  if (i <= 2) {
    band = 0; // 125Hz
  } else if (i <= 5) {
    band = 1; // 250Hz
  } else if (i <= 7) {
    band = 2; // 500Hz
  } else if (i <= 15) {
    band = 3; // 1000Hz
  } else if (i <= 30) {
    band = 4; // 2000Hz
  } else if (i <= 53) {
    band = 5; // 4000Hz
  } else if (i <= 106) {
    band = 6; // 8000Hz
  } else {
    band = 7;
  }
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band]) {
    peak[band] = dsize;
  }
}

void renderFFT(void *parameter) {
  int item = 0;
  for (;;) {
    if (uxQueueMessagesWaiting(queue) > 0) {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++) {
        peak[band] = 0;
      }

      // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
      for (int i = 2; i < (SAMPLES / 2); i++) {
        if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
        }
      }

      // Release handle
      xQueueReceive(queue, &item, 0);

      uint8_t intensity;

      FastLED.clear();
      FastLED.setBrightness(BRIGHTNESS);
      for (byte band = 0; band < NUM_BANDS; band++) {
        intensity = map(peak[band], 1, amplitude, 0, 8);

        for (int i = 0; i < 8; i++) {
          leds[getLedIndex(7 - i, 7 - band)] = (i >= intensity) ? CHSV(0, 0, 0) : CHSV(i * 16, 255, 255);
        }
      }

      FastLED.show();

      if ((millis() - lastVisualizationUpdate) > 1000) {
        log_e("Fps: %f", visualizationCounter / ((millis() - lastVisualizationUpdate) / 1000.0));
        visualizationCounter = 0;
        lastVisualizationUpdate = millis();
        hueOffset += 5;
      }
      visualizationCounter++;
    }
  }
}

void drawIcon(const uint32_t *icon) {
  // As this is invoked from loop() we can achieve a pulsing effect by
  // changing the brightness with every invocation. Goal: make it look
  // like a human breathing pattern (or like a contracting heart).
  //
  // The human breathing algorithm appears to be e^sin(x).
  // Plotted: https://www.wolframalpha.com/input/?i=e%5Esin%28x%29
  // Minimum & maximum amplitude are as follows:
  // min(e^sin(x)) = e^-1 = 1/e = 0.36787944
  // max(e^sin(x)) = e^1  = e   = 2.71828182
  //
  // Hence, in order for the minimum to be 0 you need to offset the
  // function by 1/e: e^sin(x) - 1/e. The maximum of this function thus
  // becomes e - 1/e = 2.35040238
  //
  // To map this to a (brightness) range of [0, 100]:
  // -> multiply each value with 100/(e - 1/e): 42.54590641
  // Plotted: https://www.wolframalpha.com/input/?i=%28e%5Esin%28x%29+-+1%2Fe%29+*+100%2F%28e+-+1%2Fe%29
  //
  // Multiply x (i.e. the current time) by any value to adjust the frequency.
  //
  // Inspiration: https://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
  uint8_t brightness = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 42.54590641;
  FastLED.setBrightness(brightness);
  for (int i = 0; i < LED_COUNT; i++) {
    uint32_t pixel = pgm_read_dword(icon + i);
    uint8_t red = (pixel >> 16) & 0xFF;
    uint8_t green = (pixel >> 8) & 0xFF;
    uint8_t blue = pixel & 0xFF;
    log_v("%d. %08X, %02X %02X %02X", i, pixel, red, green, blue);
    leds[getLedIndex(i % 8, i / 8)] = CRGB(green, red, blue);
  }
  delay(1);
  FastLED.show();
}

void audio_data_callback(const uint8_t *data, uint32_t len) {
  int item = 0;
  // Only prepare new samples if the queue is empty
  if (uxQueueMessagesWaiting(queue) == 0) {
    // log_e("Queue is empty, adding new item");
    int byteOffset = 0;
    for (int i = 0; i < SAMPLES; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }

    // Tell the task in core 1 that the processing can start
    xQueueSend(queue, &item, portMAX_DELAY);
  }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *) {
  log_i("Connection state changed, new state: %d", state);
  if (ESP_A2D_CONNECTION_STATE_CONNECTED == state) {
    bleDeviceConnected = true;
  } else {
    bleDeviceConnected = false;
  }
}

void playBootupSound() {
  AudioFileSourcePROGMEM *in = new AudioFileSourcePROGMEM(sound, sizeof(sound));
  AudioGeneratorAAC *aac = new AudioGeneratorAAC();
  AudioOutputI2S *out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  aac->begin(in, out);

  log_i("Playing bootup sound...");
  while (aac->isRunning()) {
    drawIcon(HEART);
    aac->loop();
  }
  aac->stop();
  log_i("...done");
}

void setup() {
  pinMode(MODE_PIN, OUTPUT);
  pinMode(pushButton, INPUT);
  digitalWrite(MODE_PIN, HIGH);

  FastLED.addLeds<WS2812B, DATA_PIN>(leds, LED_COUNT);
  playBootupSound();

  // The queue is used for communication between A2DP callback and the FFT
  // processor
  queue = xQueueCreate(1, sizeof(int));
  if (queue == NULL) {
    log_i("Error creating the A2DP->FFT queue");
  }

  // This task will process the data acquired by the Bluetooth audio stream
  xTaskCreatePinnedToCore(renderFFT,      // Function that should be called
                          "FFT Renderer", // Name of the task (for debugging)
                          10000,          // Stack size (bytes)
                          NULL,           // Parameter to pass
                          1,              // Task priority
                          NULL,           // Task handle
                          1               // Core you want to run the task on (0 or 1)
  );

  a2dp_sink.set_pin_config(pin_config);
  a2dp_sink.start((char *)DEVICE_NAME);
  // redirecting audio data to do FFT
  a2dp_sink.set_stream_reader(audio_data_callback);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
}

void loop() {
  // For some reason the audio state changed callback doesn't work properly -> need to fetch the state here.
  //
  // Otherwise you could hook this up in setup() as sketched below.
  //
  // void audio_state_changed(esp_a2d_audio_state_t state, void *){
  //   log_i("audio state: %d", state);
  // }
  // a2dp_sink.set_on_audio_state_changed(audio_state_changed);

  esp_a2d_audio_state_t state = a2dp_sink.get_audio_state();
  if (currentAudioState != state) {
    log_i("Audio state changed; new state: %d", state);
    currentAudioState = state;
  }
  switch (state) {
  // Unclear how stopped and remote suspend really differ from one another. In
  // ESP32-A2DP >= v1.6 we seem to be getting the later when the client stops
  // audio playback.
  case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
  case ESP_A2D_AUDIO_STATE_STOPPED:
    if (bleDeviceConnected) {
      if (devicePlayedAudio) {
        drawIcon(PAUSE);
      } else {
        drawIcon(BLE);
      }
    } else {
      drawIcon(HEART);
    }
    break;
  case ESP_A2D_AUDIO_STATE_STARTED:
    devicePlayedAudio = true;
    break;
  }
}
