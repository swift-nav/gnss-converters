# Novatel Support
Functionality and tools for processing Novatel data.
Compilation of these tools is gated on the `nov2sbp_BUILD` CMake option.

When creating a new build, enable Novatel support like so:
    cmake -Dnov2sbp_BUILD=true <project_dir>

# Targets
* libnovatel-parser
    - Functionality for extracting data from Novatel binary messages.
    - No external dependencies.
    - Suitable for firmware use.
* nov2sbp
    - Convert Novatel binary bytes to SBP bytes. e.g.: `cat novatel.bin | nov2sbp > novatel.sbp`
    - Relies heavily on copy-pasted code lifted from Piksi Firmware.
    - Depends on libswiftnav for some types and helper functions.
* test-novatel-parser
    - Invoke as `make run_test_novatel_parser`
    - Run unit tests on the novatel parser.

