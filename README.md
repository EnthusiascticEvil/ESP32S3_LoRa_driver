# ESP32S3 LoRa Driver (ESP-IDF, Pure C)

A minimal driver and examples for working with LoRa on the ESP32-S3 using ESP-IDF.  
The code is written in pure C for easy integration into ESP-IDF projects.

## Repository Structure

- `lora_sender/` — LoRa sender code and files for ESP-IDF (C)  
- `lora_receiver/` — LoRa receiver code and files for ESP-IDF (C)  

## Description

This project contains a driver and basic examples for working with a LoRa module on the ESP32-S3 platform using ESP-IDF.  
The code is written in pure C, without Arduino-style or C++.

The development was done using the **Heltec LoRa 32 V3** board with the built-in **SX1262** chip.

This driver is based on and adapted from the original Semtech SX126x drivers.

## Development Environment

This project was developed and tested on Windows using ESP-IDF.

ESP-IDF supports Windows, Linux, and macOS, so the project should work on these platforms as well,  
but it has not been explicitly tested outside of Windows.

For ESP-IDF installation and setup, see:  
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

## Usage

1. Connect the LoRa module to the ESP32-S3 according to your wiring (if using a different board).  
2. Open one of the examples in the `lora_sender` or `lora_receiver` folders.  
3. Build and flash the code using ESP-IDF.  
4. Use for sending and receiving LoRa messages.

## License

This project is licensed under the MIT License.  
Feel free to use and modify.

## Project Status

This project provides a basic, minimal LoRa driver and example code for ESP32-S3 using ESP-IDF.  
It is designed to be simple and easy to understand, serving as a foundation for further development or integration.

No advanced features or optimizations are included — just a straightforward working implementation.
