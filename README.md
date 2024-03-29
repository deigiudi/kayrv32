# KayRV32
[![Lint](https://github.com/deigiudi/kayrv32/actions/workflows/lint.yml/badge.svg)](https://github.com/deigiudi/kayrv32/actions/workflows/lint.yml) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

KayRV32 is a Verilog implementation of a RISC-V RV32I based microprocessor (classic 5-stage, in-order, single issue). At this point this project is in a very early state and nothing is working. Working In Progress.

1. [Project structure](#Project-structure)
2. [Dependencies](#Dependencies)
3. [Setup configuration](#Setup-configuration)
   * [Verification Flow](#Verification-flow)
   * [FPGA flow](#FPGA-flow)
   * [ASIC flow](#ASIC-flow)

## Project structure
<pre>
├── doc             # architectural description and testplan
├── fpga            # board specific HDL, constraints and bitstream
├── rtl             # Verilog HDL for the KayRV32I CPU
    ├── core                # RISC-V RV32 core
    ├── memory              # memory implementation
    └── utils               # opcodes and other helper code
├── script          # scripts to manage project and testing
    ├── vivado              # create and manage vivado project
    └── toolchain           # create and manage toolchain related scripts
├── syn             # files related to the synthesis flow
├── tb              # test related
    └── core                # contains core level tests files
        ├── basic                   # contains smoke tests
        ├── compliance              # official compatibility framework for RISC-V, RISCOF
        ├── risc-arch-test          # official riscv-arch-test repository
        └── risc-tests              # official riscv-tests repository
├── config.ini      # RISCOF configuration file
├── LICENSE
└── README.md  
</pre>


## Dependencies
This repository assumes the following to be correctly installed in your system:
- [RISCOF](https://github.com/riscv-software-src/riscof) - RISC-V Compatibility Framework
- [Sail RISC-V model](https://github.com/riscv/sail-riscv) - the reference model
- [RISC-V GCC toolchain](https://github.com/riscv/riscv-gnu-toolchain) - to compile native rv32 code

All of these can be installed by following the [RISCOF installation guide](https://riscof.readthedocs.io/en/latest/installation.html)


## Setup configuration
#### Verification flow
The KayRV32 RISC-V processor will be tested for compatibility to the RISC-V user and privileged ISA specifications with the official RISCOF compatibility framework. The [Sail RISC-V model](https://github.com/riscv/sail-riscv) is used as reference model. Files neeeded by the framework has been created according to the [RISCOF installation guide](https://riscof.readthedocs.io/en/latest/installation.html) and are stored in [/tb/core/compliance](/tb/core/compliance). In this folder, the two plugins for the framework are defined:
- **DUT:** kayrv32 in [plugin-kayrv32](/tb/compliance/plugin-kayrv32)
- **REF :** sail_cSim in [plugin-sail_cSim](/tb/compliance/plugin-sail_cSim)

The RISCOF [config.ini](/config.ini) located in the root folder is used to configure the two plugins while the official [RISC-V architecture tests](https://github.com/riscv-non-isa/riscv-arch-test) repository (located in [/tb/core/risc-arch-test](/tb/core/risc-arch-test)) provides test cases for all (ratified) RISC-V ISA extensions (user and privilege ISA). 

#### FPGA flow
This section is temporarly empty.

#### ASIC flow
This section is temporarly empty.
