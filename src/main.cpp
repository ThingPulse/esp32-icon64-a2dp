#include "BluetoothA2DPSink.h"
#include <arduinoFFT.h> 

#define NUM_BANDS  8
#define READ_DELAY 50
#define USE_RANDOM_DATA false

#include <WS2812FX.h>        //https://github.com/kitesurfer1404/WS2812FX
#include "custom/VUMeter.h"  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

BluetoothA2DPSink a2dp_sink;

#define I2S_DOUT      25
#define I2S_BCLK      26
#define I2S_LRC       22
#define MODE_PIN       33

arduinoFFT FFT = arduinoFFT();

#define SAMPLES 512              //Must be a power of 2
#define SAMPLING_FREQUENCY 40000 //Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT.

// LED Settings
#define LED_COUNT 64
#define LED_PIN     32


WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


bool isFilterActivated = false;
uint32_t lastButtonPressed = 0;
int pushButton = 39;

int amplitude = 100;
unsigned int sampling_period_us;
unsigned long microseconds;
byte peak[] = {0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime, oldTime;
bool isSampleReady = false;
bool isSampleProcessed = true;

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
};

uint32_t lastCalc = 0;

void displayBand(int band, int dsize) {
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band])
  {
    peak[band] = dsize;
  }
}

void renderFFT(void * parameter){
  for(;;){ // infinite loop
    if (isSampleReady) {
      isSampleProcessed = false;
      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
      for (int i = 2; i < (SAMPLES / 2); i++)
      { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
        if (vReal[i] > 2000)
        { // Add a crude noise filter, 10 x amplitude or more
          if (i <= 2)
            displayBand(0, (int)vReal[i] / amplitude); // 125Hz
          if (i > 3 && i <= 5)
            displayBand(1, (int)vReal[i] / amplitude); // 250Hz
          if (i > 5 && i <= 7)
            displayBand(2, (int)vReal[i] / amplitude); // 500Hz
          if (i > 7 && i <= 15)
            displayBand(3, (int)vReal[i] / amplitude); // 1000Hz
          if (i > 15 && i <= 30)
            displayBand(4, (int)vReal[i] / amplitude); // 2000Hz
          if (i > 30 && i <= 53)
            displayBand(5, (int)vReal[i] / amplitude); // 4000Hz
          if (i > 53 && i <= 200)
            displayBand(6, (int)vReal[i] / amplitude); // 8000Hz
          if (i > 200)
            displayBand(7, (int)vReal[i] / amplitude); // 16000Hz
        }
      }
      if (millis() % 4 == 0)
      {
        for (byte band = 0; band < NUM_BANDS; band++)
        {
          if (peak[band] > 0)
            peak[band] /= 2;
        }
      } // Decay the peak
      for (byte band = 0; band < NUM_BANDS; band++)
      {
        uint16_t value = peak[band];
        vuMeterBands[band] = value < 1 ? 0 : map(value, 1, amplitude, 0, 255);
        Serial.print(value);
        Serial.print("\t");
      }
      Serial.println();

      //ws2812fx.service();
      isSampleProcessed = true;
      isSampleReady = false;
    }
    log_d("Semaphores: %d, %d", isSampleReady, isSampleProcessed);
  }
}

void audio_data_callback(const uint8_t *data, uint32_t len) {
  if (isFilterActivated && isSampleProcessed) {
    isSampleReady = false;
    for (int i = 0; i < SAMPLES; i++){
      vReal[i] = data[i]; // A conversion takes about 1mS on an ESP8266
      vImag[i] = 0;
    }

    lastCalc = millis();
    isSampleReady = true;

  }
  a2dp_sink.audio_data_callback(data, len);
}

void onDataReceived() {
  log_d("Hello");
}

void setup() {
    pinMode(MODE_PIN, OUTPUT);
    pinMode(pushButton, INPUT);
    digitalWrite(MODE_PIN, HIGH);

    xTaskCreatePinnedToCore(
      renderFFT,      // Function that should be called
      "Render the FFT",    // Name of the task (for debugging)
      2000,               // Stack size (bytes)
      NULL,               // Parameter to pass
      1,                  // Task priority
      NULL,               // Task handle
      1          // Core you want to run the task on (0 or 1)
    );

    a2dp_sink.set_pin_config(pin_config);
    a2dp_sink.start("ThingPulse-Icon64");
    //a2dp_sink.set_on_data_received(onDataReceived);
    esp_a2d_sink_register_data_callback(audio_data_callback);

    ws2812fx.init();
    ws2812fx.setBrightness(32);

    // setup the custom effect
    //uint32_t colors[] = {GREEN, YELLOW, RED};
    //uint8_t vuMeterMode = ws2812fx.setCustomMode(F("VU Meter"), vuMeter);
    //ws2812fx.setSegment(0, 0, LED_COUNT-1, vuMeterMode, colors, READ_DELAY, NO_OPTIONS);

    ws2812fx.start();
}

uint8_t counter = 0;

void loop() {
  int buttonState = digitalRead(pushButton);
  if (!buttonState && millis() - lastButtonPressed > 1000) {
    isFilterActivated = !isFilterActivated;
    lastButtonPressed = millis();
    counter++;
    ws2812fx.setMode(counter % 60);
  }
  ws2812fx.service();
}