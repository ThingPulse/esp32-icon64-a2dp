#include "BluetoothA2DPSink.h"
#include <arduinoFFT.h> 
#include <SmartLeds.h>
#include "icons.h"



// Audio Settings
#define I2S_DOUT      25
#define I2S_BCLK      26
#define I2S_LRC       22
#define MODE_PIN       33

// LED Settings
#define LED_COUNT   64
#define LED_PIN     32
#define CHANNEL     0

// FFT Settings
#define NUM_BANDS  8
#define SAMPLES 512              
#define SAMPLING_FREQUENCY 44100 

#define DEVICE_NAME "ThingPulse-Icon64"

arduinoFFT FFT = arduinoFFT();
BluetoothA2DPSink a2dp_sink;
SmartLed leds( LED_WS2812B, LED_COUNT, LED_PIN, CHANNEL, DoubleBuffer );

int pushButton = 39;

float amplitude = 200.0;

int32_t peak[] = {0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];

double brightness = 0.25;

QueueHandle_t queue;

int16_t sample_l_int;
int16_t sample_r_int;
float in;

uint32_t animationCounter = 0;

int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
};

uint8_t hueOffset = 0;

bool hasDevicePlayedAudio = false;

uint8_t getLedIndex(uint8_t x, uint8_t y) {
  //x = 7 - x;
  if (y % 2 == 0) {
    return y * 8 + x;
  } else {
    return y*8 + (7 - x);
  }
}

void createBands(int i, int dsize) {
  uint8_t band = 0;
  if (i <= 2) {
    band =  0; // 125Hz
  } else if (i <= 5) {
    band =   1; // 250Hz
  } else if (i <= 7)  {
    band =  2; // 500Hz
  } else if (i <= 15) {
    band =  3; // 1000Hz
  } else if (i <= 30) {
    band =  4; // 2000Hz
  } else if (i <= 53) {
    band =  5; // 4000Hz
  } else if (i <= 200) {
    band =  6;// 8000Hz
  } else {
    band = 7;
  }
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band])
  {
    peak[band] = dsize;
  }
}

void renderFFT(void * parameter){
  int item = 0;
  for(;;) {
    if (uxQueueMessagesWaiting(queue) > 0) {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++) {
        peak[band] = 0;
      }

      for (int i = 2; i < (SAMPLES / 2); i++) { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
        if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
        }
      }

      // Release handle
      xQueueReceive(queue, &item, 0);

      uint8_t vuMeterBands[NUM_BANDS];
      
      Hsv black = Rgb(0x00, 0x00, 0x00);
      black.v = black.v * brightness;
      for (byte band = 0; band < NUM_BANDS; band++) {
        vuMeterBands[band] = map(peak[band], 1, amplitude, 0, 8);

        for (int i = 0; i < 8; i++) {
          Hsv color = Hsv(Rgb(0x00, 0xFF, 0x09));
          color.h = i * 32;
          color.v = color.v * brightness;
          leds[getLedIndex(7 - i, 7 - band)] = (i >= vuMeterBands[band]) ? black : color;
        }
      }
      // Take the message out of the queue
      leds.wait();
      leds.show();

      

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
  animationCounter++;
  leds.wait();
  for (int i = 0; i < 64; i++) {
    uint32_t pixel = pgm_read_dword(icon + i);
    byte red = (pixel >> 16) & 0xFF;
    byte green = (pixel >> 8) & 0xFF;
    byte blue = pixel & 0xFF;
    Hsv hsv = Hsv(Rgb(red, green, blue));
    log_v("%d. %08X, %02X %02X %02X", i, pixel, red, green, blue);
    hsv.v = hsv.v * (0.1 + sin(animationCounter / 40.0) * 0.06);
    leds[getLedIndex(i % 8, i / 8)] = hsv;
  }

  leds.show();


}

void audio_data_callback(const uint8_t *data, uint32_t len) {
  int item = 0;
  // Only prepare new samples if the queue is empty
  if (uxQueueMessagesWaiting(queue) == 0) {
    //log_d("Queue is empty, adding new item");
    int byteOffset = 0;
    for (int i = 0; i < SAMPLES; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset +2)));
      in = (sample_l_int + sample_r_int) / 2.0f;
      vReal[i] = in; 
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }

    // Tell the task in core 1 that the processing can start
    xQueueSend(queue, &item, portMAX_DELAY);
  }
  // pass the data to the i2s sink
  a2dp_sink.audio_data_callback(data, len);
}

void setup() {
    pinMode(MODE_PIN, OUTPUT);
    pinMode(pushButton, INPUT);
    digitalWrite(MODE_PIN, HIGH);

    // The queue is used for communication between A2DP callback and the FFT processor
    queue = xQueueCreate( 1, sizeof( int ) );
    if(queue == NULL){
      Serial.println("Error creating the queue");
    }

    // This task will process the data acquired by the 
    // Bluetooth audio stream
    xTaskCreatePinnedToCore(
      renderFFT,      // Function that should be called
      "FFT Renderer",    // Name of the task (for debugging)
      10000,               // Stack size (bytes)
      NULL,               // Parameter to pass
      1,                  // Task priority
      NULL,               // Task handle
      1          // Core you want to run the task on (0 or 1)
    );

    a2dp_sink.set_pin_config(pin_config);
    a2dp_sink.start((char*) DEVICE_NAME);
    // redirecting audio data to do FFT
    esp_a2d_sink_register_data_callback(audio_data_callback);

}


void loop() {
  esp_a2d_audio_state_t state = a2dp_sink.get_audio_state();
  switch(state) {
      case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND: 
        if (hasDevicePlayedAudio) {
          drawIcon(PAUSE);
        } else {
          drawIcon(HEART);
        }
        break;
      case ESP_A2D_AUDIO_STATE_STOPPED:
        drawIcon(BLE); 
        break;
      case ESP_A2D_AUDIO_STATE_STARTED:
        hasDevicePlayedAudio = true;
        break;
  }


}