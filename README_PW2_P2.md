Group 30:
Jan Juraszek
Arnaud Buchel 357744

Our Verilog modules (profileCi module and counter module) can be found in modules>profileCi>verilog
We also modified the or1420SingleCore.v module (systems>singleCore>verilog) to include our profiling module.
For the software, we modified the grayscale.c file.


testestestest







# [*CS-476: Embedded System Design](https://edu.epfl.ch/coursebook/en/embedded-system-design-CS-476)
This repository contains the sources of the virtual prototype as used in the course cs476 Embedded System Design.

The virtual prototype contains:
* virtualprototype/modules: Here all verilog modules of the virtual prototype are stored including their documentation.
* virtualprototype/programs: Here you find two example programs that can be run on the virtual prototype.
* virtualprototype/systems: Here you find the top-level of the single-core version of the virtual prototype and the synthesis scripts.

**Important:** The CPU used in the virtual-prototype is an *OR1420*, an in house developed OpenRISC CPU.

