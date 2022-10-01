# Example: Basic Chip32
This example demonstrates basic use of Chip32 to load a small descriptor file, parsing its header, and use it to make decisions about what images to load onto the screen.
The display is split into 4 horizontal regions. One of three possible images can be loaded into each of the 4 regions. Additional error handling is present to prevent loading invalid data.
The Chip32 program is smart enough to handle warm reloads, where it puts the core back into reset before loading new data, and takes it out of reset when complete. The bitstream is only loaded once on the first boot.

* Chip32-assisted asset load
* SDRAM controller
* Video generation
* Reload assets during runtime via the Core Settings menu



## Legal
Analogue’s Development program was created to further video game hardware preservation with FPGA technology. Analogue does not support or endorse the use of infringing content.

Analogue Developers have access to Analogue Pocket I/O’s so Developers can utilize cartridge adapters or interface with other pieces of original or bespoke hardware to support legacy media.
