# KayRV32
KayRV32 is a Verilog implementation for a RISC-V RV32I based microprocessor (classic 5-stage, in-order, single issue)

At this point this project is in a very early state and nothing is working. WIP.

1. [Project structure](#Project-structure)
2. [Setup configuration](#Setup-configuration)
3. [Dependencies](#Dependencies)

## Project structure
<pre>
- ./doc    - graphical view of the module
- ./fpga   - board specific HDL, constraints and bitstream
- ./rtl    - HDL for the KayRV32I CPU
- ./script - scripts for building and testing in vivado
- ./syn    - files related to the synthesis flow
- ./tb     - basic HDL test benches
- ./tb/compliance     - files for the official compatibility framework for RISC-V, RISCOF
- ./tb/risc-arch-test - tests from the official riscv-arch-test repository
- ./tb/risc-tests     - tests from the official riscv-tests repository
</pre>

## Setup configuration
The KayRV32 RISC-V processor has been tested for compatibility to the RISC-V user and privileged ISA specifications with the official RISCOF compatibility framework. The [Sail RISC-V model](https://github.com/riscv/sail-riscv) is used as reference model. Files neeeded by the framework has been created according to the [RISCOF installation guide](https://riscof.readthedocs.io/en/latest/installation.html) and are stored in [/tb/compliance](/tb/compliance). In this folder, the two plugins for the framework are defined:
- **DUT:** kayrv32 in [plugin-kayrv32](/tb/compliance/plugin-kayrv32)
- **REF :** sail_cSim in [plugin-sail_cSim](/tb/compliance/plugin-sail_cSim)

The RISCOF [config.ini](/config.ini) located in the root folder is used to configure the two plugins while the official [RISC-V architecture tests](https://github.com/riscv-non-isa/riscv-arch-test) repository (located in [/tb/risc-arch-test](/tb/risc-arch-test)) provides test cases for all (ratified) RISC-V ISA extensions (user and privilege ISA). 

## Dependencies
This repository assumes the following to be correctly installed in your system:
- [RISCOF](https://github.com/riscv-software-src/riscof) - RISC-V Compatibility Framework
- [Sail RISC-V model](https://github.com/riscv/sail-riscv) - the reference model
- [RISC-V GCC toolchain](https://github.com/riscv/riscv-gnu-toolchain) - to compile native rv32 code

All of these can be installed by following the [RISCOF installation guide](https://riscof.readthedocs.io/en/latest/installation.html)
