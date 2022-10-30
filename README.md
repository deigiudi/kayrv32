# KayRV32
KayRV32 is a Verilog implementation for a RISC-V RV32I based microprocessor (classic 5-stage, in-order, single issue)

At this point this project is in a very early state and nothing is working. WIP.

## Project structure
- ./doc - graphical view of the module
- ./fpga - board specific HDL, constraints and bitstream
- ./rtl - HDL for the KayRV32I CPU
- ./script - scripts for building and testing in vivado
- ./syn - files related to the synthesis flow
- ./tb - basic HDL test benches
- ./tb/compliance - files related to RISCOF validation process
- ./tb/risc-arch-test - tests from the official riscv-arch-test repository

## Setup configuration
This project is compliant to the RISCOF compatibility process to test the KayRV32 RISC-V Processor for compatibility to the RISC-V user and privileged ISA specifications. The Sail RISC-V model is used as reference model. 
