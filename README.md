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
| GPIO3 (Pin 5)          | DI / MISO |
| GPIO4 (Pin 6)          | DO / MOSI |
| GPIO5 (Pin 7)          | CS |
| GPIO6 (Pin 9)          | CLK |

Credits
-------
Most of the C code comes from [8086net/pico-sexa-uart-bridge](https://github.com/8086net/pico-sexa-uart-bridge)
