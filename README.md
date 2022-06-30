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

```
$ sudo apt-get install \
  build-essential \
  debhelper \
  libboost-system-dev \
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

## Third-party code

Third-party source code included in libs/:

 * fec - from Phil Karn's fec-3.0.1 library (see fec/README)
 * json.hpp - JSON for Modern C++ v3.5.0 - https://github.com/nlohmann/json
