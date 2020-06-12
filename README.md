# Train Movement Measurer

![Photo of completed project on my short test track](Photo.jpg)

This is a project to to measure the scale speed and acceleration of a model train moving along a piece of straight track. I hope I've managed to document it such that someone just starting out with their [Arduino](https://www.arduino.cc/) adventure can follow along and create their own. If you're not a beginner you'll likely want to skim through rather than completely read this readme. The timings required can be made either by using three gates (detecting a train being a specific spot) or four blocks (detecting a train being in a specific part of the track). You can choose to only measure speed, in which case you need one less detector.

[Why?](#why)\
[The Physics & Maths](#the-physics-maths)\
[The Electronics & Tunnel](#the-electronics-tunnel)\
[Options](#options)

---

## Why?

By measuring the scale speed of a train you're able to more easily speed match across a fleet of [DCC](https://wikipedia.org/wiki/Digital_Command_Control "Digital Command Control on Wikipedia") equipped locomotives, or learn what a realistic running speed looks like. By also measuring acceleration you're able to automate the speed matching (if you're feeling adventurous) by only paying attention to readings once the train has clearly reached the desired speed step or learn how not to make your passengers spill their coffee.

## The Physics & Maths

We're going to assume that the track is a 20cm long straight length (curves are possible you just need to be careful with measuring how far the train travels between different triggers - the maths is then the same). Physicists normally love simplifying things down to spheres in a vacuum, however for us it's simpler to consider our train to be a solid cube travelling along a straight line (our track). The fact the front of our train isn't exactly a cube doesn't matter as assuming the detectors are all the same then they will be triggered when the same part of the train is at the spot we care about.

As we learnt and school speed is calculated by dividing distance travelled by time taken.\
_s = d ÷ t_\
To keep things simple we'll stick to metric so we end up with a measurement in m/s (meters per second). Since we know the distance between our triggers it's as simple is timing how long elapses between two triggers being triggered.
In the same way as speed measures the rate at which distance changes acceleration measures the rate at which speed changes, since we're working in metric this is in meters per second per second or m/s² (in the following formula Δ represents the change in a value).\
_a = Δs ÷ t_\
At this point we can do some conversions to get from m/s and m/s² into something a bit more meaningful:
* To get Km/h or Km/h/s multiply by 3.6 × scale
* To get mph or mph/s multiply by 2.236936 × scale

Now we now the basics, let's discover how to use three detectors to measure three speeds and one acceleration:\
d₁ is the distance between the first two triggers.\
d₂ is the distance between the last two triggers.\
d is the total distance (_d₁ + d₂_).\
t₁ is the time between the first two triggers.\
t₂ is the time between the last two triggers.\
t is the total time (_t₁ + t₂_).\
s₁ is the speed between the first two triggers.\
s₂ is the speed between the last two triggers.\
s is the speed between the first and last detector.\
a is the acceleration between the first and last  detector.

From the above we can easily calculate our speeds using:\
_s₁ = d₁ ÷ t₁_\
_s₂ = d₂ ÷ t₂_\
_s = d ÷ t_\
From here we have to understand that if the train is accelerating at a constant rate (which we're going to assume it is to keep things simple) then the speeds s₁ and s₂ occur at the times ½t₁ and ½t₂ (that is to say half way through each gap between detectors). We can therefore calculate the speed change and acceleration using:\
_Δs = s₂ - s₁_\
_a = Δs ÷ ½t_

![](ReadmeMathsDiagram.svg)

There are two methods of measuring the time, depending on which style of train detection makes the most sense for your use case. I'll only describe the measurements for a train moving left to right but the code can also handle a train running right to left.\
Either we can setup three gates to detect when the train is a particular point of the track. Timer 1 is started when gate 0 is triggered and stopped when gate 1 is triggered. Timer 2 is started when gate 1 in triggered and stopped when gate 2 is triggered.\
Or we can setup four blocks of track to detect when a train is present in them (we only actually care about the length of two of them). Timer 1 is started when track 1 becomes occupied and stopped when track 2 becomes occupied. Timer 2 is started when track 2 is occupied and stopped when track 3 is occupied.
If you're only interested in measuring speed the comments in the code give you various options.

## The Electronics & Tunnel

The electronics are rather simple an Arduino provides the brains whilst an OLED display and NeoPixel LED stick let you know what's going on and how fast you're driving.
There are several options for detecting a train's position for the calculations, these are discussed in [Electronics/README.md](Electronics/README.md).

As you may be able to make out from the photo at the top I opted for using three light gates attached to a platic tunnel. A light gate is simply alight shining accorss the track into a detector, when the beam is broken we knoe the train has arrived at that point.
If you wish to use this method then see  [Tunnel/README.md](Tunnel/README.md).

## The Code

The code provides some test modes which can be used to troubleshoot the hardware, see the comments in the code for further details. The code may need some changes before you compile and upload it to the Arduino, follow the comments in the code file.

If you've built this exactly as I did you should only need to change one line (which tells the Arduino what scale to use). If you've kept the electronics the same but changed how far apart the light gates are you'll need to make 2 more changes. There are some other things you can tweak in the code such as how long the results are displayed on the screen for. Depending on your display you may need to make some more changes (probably the I2C address). You can also easily change the number of LEDs in the strip should for example you want to use a semi circle or longer stick with more LEDs - in which case you'll want to change the list of colours and what speeds they light for the LEDs.

The code goes through a number of steps when the Arduino receives power:
1. **Setup** - The Arduino gets itself and he screen up and going.
2. **Check** - That the sensors are clear, if not you'll get an error and the display and LEDs will be used to show you which sensors are clear/blocked.
3. **Wait for and time a train**:
    - Using gates:
       1. Wait for the light gate at either end to be blocked - We save when this happened, since we know what direction the train is travelling in we keep track of what the exiting light gate will be too. The LEDs are used to show the direction of travel and that the first time has been saved. The display is updated to show that a train is being timed.
       2. Wait for the middle light gate to be blocked - We save when this happened. The LEDs are used to show that this time has been saved. Skipped if measuring speed only.
       3. Wait for the final light gate to be blocked - We save when this happened. The LEDs are used to show that this time has been saved. We now have gathered all the data we need.
   - Using track occupancy:
       1. Wait for track 1 or 2 to become occupied - We save when this happened, since we now know the direction the train is travelling in we keep track of what the next and final track section will be (either [2 then 3] or [1 then 0]). The LEDs are used to show thw direction of travel and that the first time has been saved. The display is updated to show that a train is being timed.
       2. Wait for the other of track 1 or 2 to become occupied - We save when this happened. The LEDs are used to show that this time has been saved. Skipped if measuring speed only.
       3. Wait for either track 0 or 3 (as appropriate for the train's direction) to become occupied - We save when this happened. The LEDs are used to show that this time has been saved. We now have gathered all the data we need.
4. **Calculate** - We now use the time differences to calculate two speeds and an acceleration (or just the speed). These are then scaled and displayed on the display. The LEDs now show a bar of varying colours - the more LEDs the faster the scale speed. All the calculated values are output to the serial port too (see [SERIAL_PORT.md](SERIAL_PORT.md)).
5. **Wait** - After the configurable time delay (defaults to 15 seconds) we start checking that the train is completly left the area we're watching, when it has we go back to step 2.

## Options

### Gates Without the Light Beam

The only function which the light gates serve is to detect when the train is at specific spots on the track. This could alternativly be achieved in a number of different ways e.g. a button operated by a person or the train's wheels. You will need to make your own adjustments to the circuitry (see [Electronics/LightGates.md](Electronics/LightGates.md)). In addition you'll need to make the following changes in the code:
* Make sure that you configure TRIGGER_PINS to contain the Arduino pins connected to your detectors
* Remove/comment the "#define GATE_LED_PIN" line
* Ensure you set TRIGGER_DETECT correctly

### Track Occupancy Detection

There are a number of ways in which block detection work on a model railway, as such the interfacing between this project and the many available systems will vary. For this reason the electronics are left to the user to design (see [Electronics/Blocks.md](Electronics/Blocks.md)). In addition you'll need to make the following changes in the code:
* Make sure that you configure TRIGGER_PINS to contain the Arduino pins connected to your detectors
* Remove/comment the "#define GATE_LED_PIN" line
* Ensure you set TRIGGER_DETECT correctly
* Make sure that "#define TRIGGER_BLOCKS" is present

### Measure Speed Only

With the current code this is easily achieved by:
* Changing "#undef VELOCITY_ONLY" to "#define VELOCITY_ONLY"
* Making sure your detectors are numbered as the comments in the code describe

### Change the Logo

See [Logos/README.md](Logos/README.md).
