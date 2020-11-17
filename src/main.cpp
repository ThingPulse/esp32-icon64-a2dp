#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;

#define I2S_DOUT      25
#define I2S_BCLK      26
#define I2S_LRC       22
#define MODE_PIN       33

static int16_t sample_l_int = 0;
static int16_t sample_r_int = 0;
static float sample_l_float = 0.0f;
static float sample_r_float = 0.0f;
static float in = 0.0f;
static float out = 0.0f;
static float out2 = 0.0f;
static float biquad1_z1 = 0.0f;
static float biquad1_z2 = 0.0f;
static float biquad2_z1 = 0.0f;
static float biquad2_z2 = 0.0f;
static float biquad3_z1 = 0.0f;
static float biquad3_z2 = 0.0f;
static float biquad4_z1 = 0.0f;
static float biquad4_z2 = 0.0f;
static float biquad5_z1 = 0.0f;
static float biquad5_z2 = 0.0f;
uint32_t s_pkt_cnt = 0;

bool isFilterActivated = false;
uint32_t lastButtonPressed = 0;
int pushButton = 39;

static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
};

void audio_data_callback(const uint8_t *data, uint32_t len) {
  if (isFilterActivated) {
    size_t bytes_written;
    uint8_t *mydat = (uint8_t*) malloc(len);
    if(mydat == NULL)
    {
        ESP_LOGI(BT_AV_TAG, "Not enough memory available");
    }
    else
    {
        for (uint32_t i = 0; i < len; i+=4)
        {
            sample_l_int = (int16_t)((*(data + i + 1) << 8) | *(data + i));
            sample_r_int = (int16_t)((*(data + i + 3) << 8) | *(data + i +2));
            sample_l_float = (float)sample_l_int / 0x8000;
            sample_r_float = (float)sample_r_float / 0x8000;
            in = (sample_l_float + sample_r_float) / 2.0f;
            //Audio processing Left Channel
            //HP 130Hz
            out = in * (0.9869865272227986f) + biquad1_z1;        //out = in * a0 + z1;
            biquad1_z1 = in * (-1.9739730544455971f) + biquad1_z2 - (-1.9738037472862642f) * out;    //z1 = in * a1 + z2 - b1 * out;
            biquad1_z2 = in * (0.9869865272227986f) - (0.9741423616049301f) * out;        //z2 = in * a2 - b2 * out;
            //Peak Fc 10095, Q 0.478, 6dB Gain
            out2 = out * (1.5066355805385152f) + biquad5_z1;        //out = in * a0 + z1;
            biquad5_z1 = out * ( -0.12972459720746804f) + biquad5_z2 - (-0.12972459720746804f) * out2;    //z1 = in * a1 + z2 - b1 * out;
            biquad5_z2 = out * (-0.5247301530319133f) - (-0.018094572493397725f) * out2;        //z2 = in * a2 - b2 * out;
            out = out2 * 0.4f;
            //End of audio processing
            int32_t a;
            a = floor(32768 * out);
            if (a>32767) {a = 32767;}
            if (a<-32768) {a = -32768;}
            sample_l_int = (int16_t)a;
            *(mydat + i + 1) = (uint8_t) ((sample_l_int >> 8) & 0xFF);
            *(mydat + i) = (uint8_t) (0xFF & sample_l_int);
            //end of left Channel
            
            //Audio processing Right Channel
            //HP 50Hz 1 Q=0.54119610
            out = in * (0.993448957683901f) + biquad3_z1;        //out = in * a0 + z1;
            biquad3_z1 = in * (-1.986897915367802f) + biquad3_z2 - (-1.9868727071697347f) * out;    //z1 = in * a1 + z2 - b1 * out;
            biquad3_z2 = in * (0.993448957683901f) - (0.9869231235658689f) * out;        //z2 = in * a2 - b2 * out;
            //HP50Hz 2 Q=1.306563
            out2 = out * ( 0.9972686246697485f) + biquad4_z1;        //out = in * a0 + z1;
            biquad4_z1 = out * (-1.994537249339497f) + biquad4_z2 - (-1.9945119442195687f) * out2;    //z1 = in * a1 + z2 - b1 * out;
            biquad4_z2 = out * (0.9972686246697485f) - (0.9945625544594253f) * out2;        //z2 = in * a2 - b2 * out;
            //LP 130Hz
            out = out2 * (0.00008465357966651423f) + biquad2_z1;        //out = in * a0 + z1;
            biquad2_z1 = out2 * (0.00016930715933302846f) + biquad2_z2 - (-1.9738037472862642f) * out;    //z1 = in * a1 + z2 - b1 * out;
            biquad2_z2 = out2 * (0.00008465357966651423f) - (0.9741423616049301f) * out;        //z2 = in * a2 - b2 * out;
            out = out * 1.0f;
            //End of audio processing
            a = floor(32768 * out);
            if (a>32767) {a = 32767;}
            if (a<-32768) {a = -32768;}
            sample_r_int = (int16_t)a;
            *(mydat + i + 3) = (uint8_t) ((sample_r_int >> 8) & 0xFF);
            *(mydat + i + 2) = (uint8_t) (0xFF & sample_r_int);
            //end of right Channel            
        }
        a2dp_sink.audio_data_callback(mydat, len);
        //i2s_write(0, mydat, len, &bytes_written, portMAX_DELAY);
        free (mydat);
    }
    } else {
      a2dp_sink.audio_data_callback(data, len);
    }
    if (++s_pkt_cnt % 100 == 0) {
        ESP_LOGI(BT_AV_TAG, "Audio packet count %u", s_pkt_cnt);
        ESP_LOGI(BT_AV_TAG, "Process task core: %u\n", xPortGetCoreID());
    }

    
}

void onDataReceived() {
  log_d("Hello");
}

void setup() {
    pinMode(MODE_PIN, OUTPUT);
    pinMode(pushButton, INPUT);
    digitalWrite(MODE_PIN, HIGH);
    a2dp_sink.set_pin_config(pin_config);
    a2dp_sink.start("Icon64");
    //a2dp_sink.set_on_data_received(onDataReceived);
    esp_a2d_sink_register_data_callback(audio_data_callback);
}

void loop() {
  int buttonState = digitalRead(pushButton);
  if (!buttonState && millis() - lastButtonPressed > 1000) {
    isFilterActivated = !isFilterActivated;
    lastButtonPressed = millis();
  }
}