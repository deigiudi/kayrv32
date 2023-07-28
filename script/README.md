<pre>
├── vivado
    ├── createProject.sh       # create a Vivado project and start GUI. Use only once. 
    ├── openProject.sh         # open Vivado project and start GUI
    ├── restartTest.tcl        # restart a simulation and log results
    └── showResults.sh         # show VCD results from tests using GTKWave
├── setTestEnv.sh      # Set shell environment variables (needs to be run before generateHex.sh)
├── generateHex.sh     # Compile & Link riscv-tests and generate hex files to be used in tests
└── README.md  
</pre>

## External Dependencies
- ***generateHex.sh***: uses [RISC-V GCC toolchain](https://github.com/riscv/riscv-gnu-toolchain) and [riscv32-unknown-elf-elf2hex](https://github.com/sifive/elf2hex)
- ***showResults.sh*** : uses [GTKWave](https://github.com/gtkwave/gtkwave) to display results from basic tests without the need of launching the Vivado IDE
