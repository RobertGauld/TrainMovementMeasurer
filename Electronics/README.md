# Train Movement Measurer - The Electronics

The heart of the project is an Arduino Nano where our code runs, this can be powered by the onboard USB or through the screw terminal which connects to the Arduino's Vin (accepting 7-12V DC, this circuit requires upto 500mA). It uses an OLED display (connected using I2C) and an optional strip of WS2812 LEDs for output (connected to wires soldered to the breadboard).

There are a number of options for detecting trains:
- [Light Gates](LightGates.md) - Three IR LEDs shine accross the track into three IR detectors. When the train breaks the beam we know it's at that spot. The third can be omitted if you're only measuring speed.
- [Gates](Gates.md) - Three detectors allow us to know when a train is at speific spots on the track. The third can be omitted if you're only measuring speed.
- [Block Occupancy](Blocks.md) - Four sections of track are fitted with train on track detectors. When the train enters/leaves each block we know how far it travelled.
