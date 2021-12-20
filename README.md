# ThingPulse Icon64 Bluetooth Speaker

ESP32 based Bluetooth loudspeaker with 8 band spectrum analyzer. This is the stock firmware for the [ThingPulse Icon64](https://thingpulse.com/product/icon64/) devices.

[![ThingPulse Icon64](https://thingpulse.com/wp-content/uploads/2020/11/Whitebox_Heart.jpg)](https://thingpulse.com/product/icon64/)

## Demo video

[![YouTube demo](http://img.youtube.com/vi/1UpbtE98OBA/0.jpg)](http://www.youtube.com/watch?v=1UpbtE98OBA "Icon64 Bluetooth Speaker")

## LED matrix state machine

Below list briefly explains what is displayed on the "GUI" (i.e. the LED matrix) in which state. Note
that all icons are rendered in a pulsing manner.

- **no** BLE audio device connected to Icon64: heart icon (see image above)
- BLE audio device connected: Bluetooth icon
- BLE audio device connected and audio playback suspended (when once started): pause icon
- audio playing: spectrum analyzer

