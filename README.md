# Switchgrass Cattail

Code for the ESP32 that sits atop our cattails.
The ESP32:
- listens to an accelerometer (MPU6050)
- outputs readings from the accelerometer over UART
- controls WS281x lights based on values from the accelerometer

## Flashing instructions

### Setup - Install Tools

Install rust.

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install all esp-related tools, following the [Embedded Rust (no_std) on Espressif](https://docs.espressif.com/projects/rust/no_std-training/) guide:
```sh
cargo install cargo-espflash espflash
brew install llvm
```

Also install just, a command runner:

```sh
brew install just
```

### Calibrating the ESP

In order for the cattail to know which way is up, you must hold it upright and calibrate it.
Hold the ESP so that the flashing interface up, and the lights are down. Then, run 
```sh
just calibrate
```
After a minute, you should see a bunch of values printed out.
The accelerometer values should be close to x=-1, y=0, z=0.

### Flashing the ESP

To flash the ESP, run
```sh
just flash CATTAIL_ID
```
