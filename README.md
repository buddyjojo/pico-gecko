# pico-gecko
Raspberry Pi Pico based USB gecko clone

Notes
-----
- Sending data to the gc/wii is currently very unreliable.
- Only works with CLK set to the "101" frequency (all homebrew I've come across use this frequency) due to using delays instead of clocking in the data.
- Overclocks the rp2040 to 250mhz, most boards can handle this but some are [known to not](https://github.com/Noltari/pico-uart-bridge/issues/11#issuecomment-2048104347).

Pinout
------
| Raspberry Pi Pico GPIO | Function |
|:----------------------:|:--------:|
| GPIO2 (Pin 4)          | DI / MOSI |
| GPIO3 (Pin 5)          | DO / MISO |
| GPIO4 (Pin 6)          | CS |
| GPIO5 (Pin 7)          | CLK |

<img src="https://github.com/user-attachments/assets/f31dfe66-b884-448e-9230-c3edcb3adca5" width="224.75" height="379.75">
<img src="https://github.com/user-attachments/assets/1fef3ac4-c165-4bba-9962-3b80aac4da0e" width="224.75" height="379.75">

Credits
-------
Most of the C code comes from [8086net/pico-sexa-uart-bridge](https://github.com/8086net/pico-sexa-uart-bridge)
