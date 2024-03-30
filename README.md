# ST25R39 driver

WIP driver for (some) STMicro NFC readers (and maybe targets)

So far it can boot the ST25E3916(B) on a X-NUCLEO-NFC06A1 (or 08A1, they seem to be mostly the same),
measure some stuff and perform automatic antenna tuning.

## Library TODOs

Time types and conversions (timings are expressed as multiples of the 13.56MHz clock periods)

Figure out why collision detection is not working
