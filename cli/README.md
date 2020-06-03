A Multiwii Serial Protocol (MSP) CLI
===========================================

async_msp_cli is a command line program for configuring rebooting, getting status and downloading blackbox from Cleanflight, Betaflight and iNav.
This is an incomplete implementation of the MSP2 commands, with some Cleanflight, Betaflight and iNav extensions.

1. Mimic iNav, betaflight cli
2. Send MSP commands asynchronously
3. Be as fast and efficient as possible

### Available commands
```
aux                             Get all aux setting
                  set           Set aux setting

feature                         Get all features
                  Set           features(FEATURE_NAME:enable, -FEATURE_NAME:disable)
                  disable       Disable feature
                  enable        Enable feature

osd_item                        Get all osd setting
                  set           Set osd setting

blackbox          download      Download blackbox concurrently
                  downloadv2    Pull blackbox serially

map                             Get all rx map setting
                  set           Set rx map setting

mmix                            Get all mmix setting
                  set           Set mmix setting

serial                          Get all serial setting
                  set           Set serial setting

setting           list          Get all common setting
                  set           Set common setting
                  get           get common setting
                  set-all       Set all common setting

smix                            Get all smix setting
                  set           Set smix setting

config                          Get all configs
                  set           Upload all configs

reboot                          Reboot the device

OPTIONS:
    -p, --port <port>    device serial port

FLAGS:
    -r, --reboot     reboot fc
    -s, --save       settings file path
```

### Backup and restore FC configs

```bash
async_msp_cli config > /tmp/configs # dump all configs
async_msp_cli -s -r config set /tmp/configs # restore all configs from file
```

The dump has the same syntax as configurator dump command output
```
mmix 0  1.000 -1.000  1.000 -1.000
mmix 1  1.000 -1.000 -1.000  1.000
...

feature -TX_PROF_SEL
feature THR_VBAT_COMP
...

aux 0 51 6 1100 1200
aux 1 52 6 1400 1500
...

set gyro_hardware_lpf = 256HZ
set gyro_lpf_hz = 90
...

```

### Installation
Compiling from this repository also works similarly:

You can compile from source by
[installing Cargo](https://crates.io/install)
([Rust's](https://www.rust-lang.org/) package manager)
and installing `async_msp_cli` using Cargo:

```bash
cargo install --locked async_msp_cli
```

```bash
git clone git://github.com/BurntSushi/async_msp_cli
cd async_msp_cli
cargo build --release
```

Compilation will probably take a few minutes depending on your machine. The
binary will end up in `./target/release/async_msp_cli`.


Dual-licensed under MIT or the [UNLICENSE](https://unlicense.org).
