A Multiwii Serial Protocol (MSP) commands implementation for Rust
===========================================

## Introduction

An incomplete implementation of the MSP2 library, with some Cleanflight, Betaflight and iNav extensions. Allows one to cli to interact with Cleanflight, Betaflight and iNav.

# Installation

MSP is available on crates.io and can be included in your Cargo enabled project like this:

```toml
[dependencies]
async_msp_lib = "0.1.12"
```

Then include it in your code like this:

```rust
extern crate async_msp_lib;
```


##### Resources

* http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
* http://armazila.com/MultiwiiSerialProtocol(draft)v02.pdf
* https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp

License: MIT OR Apache-2.0
