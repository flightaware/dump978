# dump978-fa

This is the FlightAware 978MHz UAT decoder.

It is a reimplementation in C++, loosely based on the demodulator from
https://github.com/mutability/dump978.

For prebuilt Raspbian packages, see https://flightaware.com/adsb/piaware/install

## Overview

dump978-fa is the main binary. It talks to the SDR, demodulates UAT data,
and provides the data in a variety of ways - either as raw messages or as
json-formatted decoded messages, and either on a network port or to stdout.

skyaware978 connects to a running dump978-fa and writes json files suitable
for use by the SkyAware web map.

## Building as a package

Caution: The package build is memory-hungry. A 1GB Pi will fail to build the
package. Please build on a machine with more memory, or add swap.

```
$ sudo apt-get install \
  build-essential \
  debhelper \
  libboost-program-options-dev \
  libboost-regex-dev \
  libboost-filesystem-dev \
  libsoapysdr-dev

$ dpkg-buildpackage -b
$ sudo dpkg -i ../dump978-fa_*.deb ../skyaware978_*.deb
```

## Building from source

 1. Ensure SoapySDR and Boost are installed
 2. 'make'

## Installing the SoapySDR driver module

You will want at least one SoapySDR driver installed. For rtlsdr, try

```
$ sudo apt-get install soapysdr-module-rtlsdr
```

## Configuration

For a package install, see `/etc/default/dump978-fa` and
`/etc/default/skyaware978`.

The main options are:

 * `--sdr` specifies the SDR to use, in the format expected by
   SoapySDR. For a rtlsdr, try `--sdr driver=rtlsdr`. To select a
   particular rtlsdr dongle by serial number, try
   `--sdr driver=rtlsdr,serial=01234567`
 * `--sdr-gain` sets the SDR gain (default: max)
 * `--raw-port` listens on the given TCP port and provides raw messages
 * `--json-port` listens on the given TCP port and provides decoded messages
   in json format

Pass `--help` for a full list of options.

## Adaptive gain control

When receiving samples from an SDR (`--sdr` only), dump978-fa can adjust the
SDR gain automatically. This is a port of
[dump1090-fa's adaptive gain feature](https://github.com/flightaware/dump1090/blob/master/README.adaptive-gain.md)
and works the same way. Two independent modes are available; both are off by
default and can be enabled together:

 * `--adaptive-range` enables **dynamic range mode**: dump978 measures the
   noise floor and searches for the highest gain that still provides a target
   dynamic range (by default, 60% of the SDR's gain range in dB -
   approximately 30dB for an RTLSDR). This suits most installs.
 * `--adaptive-burst` enables **burst mode**: dump978 watches for loud,
   message-length bursts of signal that failed to decode - a sign that a
   nearby transmitter is overloading the receiver - and reduces gain when
   they arrive too often, raising it again once things quieten down.

Useful tuning options:

 * `--adaptive-min-gain <dB>` / `--adaptive-max-gain <dB>` restrict the gain
   range that adaptive gain control will use. If you know an approximately
   correct gain for your installation, setting limits around it speeds up
   convergence.
 * `--adaptive-duty-cycle <percent>` controls the fraction of received
   samples inspected (default: 50). Lower values reduce CPU use on slow
   machines at a small cost in measurement accuracy.
 * `--adaptive-range-target <dB>` overrides the target dynamic range.

Further tunables mirroring dump1090-fa's options are available; pass `--help`
for the full list. Adaptive gain control requires manual gain control of the
SDR and cannot be combined with `--sdr-auto-gain`. Gain changes and a
once-a-minute status line are logged to stderr.

## Third-party code

Third-party source code included in libs/:

 * fec - from Phil Karn's fec-3.0.1 library (see fec/README)
 * json.hpp - JSON for Modern C++ v3.5.0 - https://github.com/nlohmann/json
