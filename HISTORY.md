## 1.0.0
New Features:
  * Log file size settable from commandline
  * Improved handling of multiple raspalib restarts
  * Less logging while waiting for initial connection
  * Updated spdlog to v1.17.0

Fixes:
  * Missing ADC parameters added to DiscreteSensorMapper, fixing unhandled command errors
  * DiscreteSensorMapper previous value not cleared correctly
  * RelativeSensor values no longer requested on refresh
  * 0 values from MCU ignored for RelativeSensor
  * Incorrect 'Shiftreg Gpio' log messages
  * Build fix for latest Xcode

## 0.5.0
New Features:
  * New relative_input and discrete_input sensor types
  * Synchronous messaging from hardware to user frontend (OSC or gRPC)
  * Input and output pins merged into a common sensor concept
  * Config file validated using a JSON schema
  * New analog filter parameters
  * Updated elk-gpio-protocol
  * Bitbucket pipeline and clang-format support

Fixes:
  * Frequent crash on shutdown
  * Hang on exit
  * Only known controllers are refreshed
  * --log-flush-interval accepts floating point values
  * Removed irrelevant range parameter for analog inputs
  * macOS build fixes

## 0.4.0
New Features:
  * gRPC frontend, with RefreshAllStates and configurable host and port
  * Backend selection via config file
  * jsoncpp and liblo dependencies managed through vcpkg
  * Python based raspa_mock for testing
  * Updated elk-gpio-protocol

Fixes:
  * Previous sensor values cleared when refreshing
  * Missing mutex for gRPC controller map
  * Clearer event logging in backends
  * Compilation fixes for newer compilers
