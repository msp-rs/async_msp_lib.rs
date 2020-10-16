async-msp-lib
===========================================

An incomplete implementation of the MSP2 library, with some Cleanflight, Betaflight and iNav extensions

This crate aims to provide convenient API for MSP communication, by providing as effecient and asynchronous communication


# Usage
This library comes with ready [CLI](../tree/master/cli) and [C Bindings](../tree/master/bind) packages.
But can be used directly in your rust project .

MSP is available on crates.io and can be included in your Cargo enabled project like this:

```toml
[dependencies]
async_msp_lib = "0.1.15"
```

Then include it in your code like this:

```rust
extern crate async_msp_lib;
```

build release
```
cargo build --features suppport_int32_setting_type

cargo make --makefile ./Makefile.toml --no-workspace release-all-int32
cargo make --makefile ./Makefile.toml --no-workspace release-int32
```


##### Resources

* http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
* http://armazila.com/MultiwiiSerialProtocol(draft)v02.pdf
* https://stackoverflow.com/questions/42877001/how-do-i-read-gyro-information-from-cleanflight-using-msp

License: MIT OR Apache-2.0
