# micropython-NFC

This repository contains some experiments with NFC readers. 

First experiment was to have the MFRC522 working on the PYB board. Next thing is to move it over to the ESP8266.

# MFRC522
One of the more mature readers is the NXP MFRC522 Reader. This is a 3.3V reader with `UART`, `I2C` and `SPI` interfaces. The latest silicon v2.0 is in the `MFRC52202HN1` chip.

The reader supports:
- MIFARE Mini,
- MIFARE 1K, 
- MIFARE 4K, 
- MIFARE Ultralight, 
- MIFARE DESFire EV1 and 
- MIFARE Plus RF identification protocols.

# mfrc522.py
The code uses the `PYB` module (need to change to `machine`). Using `execfile('mfrc522.py')` to start the reading, `import MFRC522` to add the module to your program.

