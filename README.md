# esp32-tests

Code contained in components/SCServo_esp32/ is not mine, I am still researching the origin and the licence of this code.
For now consider that components/SCServo_esp32 is not under MIT license

## Getting Started

To install esp-idf use a Ubuntu 22.04 environment and clone https://github.com/hdumcke/multipass-orchestrator-configurations.git

```
multipass-orchestrator-configurations/esp-idf/build.sh
```

should create a working environment. I use a Raspberry Pi 4B with 2G of RAM as my build system

then clone this directory and:

```
cd esp32-tests
idf.py set-target esp32s3
idf.py menuconfig # enable backward compatibility for freertos
idf.py build
export ESPPORT=/dev/ttyUSB0
idf.py monitor # set ESP32 in download mode if you use a simple serial adapter
idf.py flash
```

## Status

This is work in progress and should be of no interest other than people I work with directly
