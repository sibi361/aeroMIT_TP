# Section A

---

#### 1 What are the types of motors available for multi-rotors? Explain in detail which one is the most efficient and why with an extensive comparison with the rest.

The two types of motors available for multi-rotors are:

- Brushed
- Brushless

Brushless motors are more efficient for the following reasons:

- Absence of brushes
  Except for the shaft support, the stator and rotor in a brushless motor are virtually separate entities. This reduces mechanical energy loss due to friction thus giving longer life
- Better speed control
  Since brushless motors have three input wires as compared to just one in brushed motors (excluding ground), they offer more precise control. For example while performing steep maneuvers, the sudden changes in motor speed can be performed.
- Outrunner motors (rotor rotating outside over the stator)
  It's difficult to have an efficient DC outrunner motor because of the brushes which would need to be somehow mounter on the outer periphery of the motor casing. This would make the design too bulky and prone to damage. Source: https://drones.stackexchange.com/questions/844/are-there-any-brushed-outrunner-motors

---

#### 2 How is the working of a tricopter different from that of a quadcopter? Imagine you are building a UAV for a competition; which one would you prefer? Give a detailed explanation.

A tricopter has three arms whereas a quadcopter has four. All four of a quadcopter arms have one motor each of which two spin clockwise and the other two spin counter clockwise. Even the tricopter has one motor per arm but one of the motors is made moveable on the axis of the arm, by connecting it to a servo motor. This is used for the yawn movement which otherwise would not be possible like in a quadcopter wherein the yawn movement can be done by changing the speed of the motors.

For a competition where payload carrying capacity is important we would prefer a quadcopter over a tricopter as it has more thrust and can be used to carry cameras, medicine etc. But if the competition is racing oriented or asks for longer flight time, we might go for a tricopter as it is more agile and due to the lesser number of motors it can give better battery life.

---

#### 3 Explain in detail the flight movement shown by drones on the three axes. How do they achieve those? How are these movements achieved on a fixed wing plane?

Flight movement shown by drones on the three axes are:

- Roll: movement of a drone about the longitudinal axis. The word "longitude" used here is similar to earth's longitude which means its vertical. This movement is required to make the drone go left or right. To go left the both the motors on the right of the quadcopter spin faster than those on the left and vice versa.

- Pitch: movement of a drone about a lateral axis. This movement is used to make a drone move forward or backward. To go forward, both the motors on the back of the quadcopter spin faster than those on the front and vice versa.

- Yawn: movement of the drone about the Z axis or the vertical axis. This kind of motion is used to change the heading of the drone. For example a drone might have a camera mounted on it on one side of the drone. When airborne, if the camera has to capture something on the left, the drone itself has to move ninety degrees to the left. This is yawn movement. It is achieved by making one set of diagonal propellers run at a higher speed than the rest.

On a fixed wing plane, these movements are achieved by the use of servo motors which move parts of the plane's fuselage:

- Roll: Aileron
  Behind each wing at the edge, there will be a movable flap. In order to make the plane roll, the flaps on either wings will rotate in the OPPOSITE directions. This causes a change in lift on both the wings and causes roll movement.

- Pitch: Elevator
  The back ends of each of the horizontal tail surface will be movable in the up down direction. In order to make the plane pitch up, BOTH the flaps will move upwards causing a low pressure region under the flaps and a high pressure region above it, and vice versa.

- Yaw: Rudder
  The back edge of the vertical tail fin is movable. To yawn right, it would be moved to the right and vice versa.

---

#### 4 Explain in detail the working of an ESC and the way it controls a BLDC motor.

An Electronic Speed Controller (ESC) has the following parts:

- Output wires for the motor \* 3
- Power input wires \* 2
- Signal cable \* 1
- \+ 5V cable \* 1

When the ESC is initially switched on with a Lithium Polymer (lipo) battery:

- the Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable
- it waits for a 50Hz pulse width modulated (PWM) signal on the signal wire from either a R/C receiver or a microcontroller

An ESC has three main functions:

- Making the Brushless Direct Current (BLDC) rotor rotate:
  A BLDC motor is similar to a 3 phase AC motor. Taking the example of an outrunner motor, it consists of a stator which is divided into three alike arms 120 degrees apart with copper windings on them. The spinning part is the rotor which is present over the stator. The rotor has magnets lined up throught its circumference.

When an input signal is received it shows that by making a beep sound on the motor after which raising the throttle further starts the motor. The ESC starts by providing a positive voltage, say 5V on one of the motor inputs while keeping the other two inputs at -5V, for a very short amount of time, say 100 ms. This causes the copper coil in the arm connected to that wire to get magnetised and thus attract the rotor. After this time it switches the power to another motor input making the rest two -5V. The rotor is now attracted to this other arm of the stator. And after another 100 ms has passed it repeats the process again. This subsequent switching of the voltages in the three motor inputs makes the rotor rotate. The rate of switching determines the speed of the motor and this rate is controlled by the signal wire.

- Improving motor efficiency
  When the motor rotates, due to Faraday's law a reverse current is generated in the motor cables, especially the ones which are in the Off state. The ESC "listens" to this feedback given by the motor and accordingly adjusts the current in order to optimise the motor output.

- BEC

The Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable. Not all ESCs possess BECs.

Apart from these main functions it also provides functions some secondary functions dependent on the firmware:

- Quick Braking
  Making the rotor stop suddenly by causing it to suddenly rotate in the opposite direction
- Safety features
  If the R/C throttle is not at zero when the ESC is powered on, it wouldn't let the motor start unless it's brought to zero first

---

#### 5 What are the operational frequencies of the electronic components on a quadcopter? Explain why those are chosen specifically and what are their advantages and disadvantages.

The radio control for the drone's movement is done on 2.4 Ghz frequency whereas the video transmission is usually done at 5.8 Ghz.
This is because of the following facts:

- Frequency is inversely proportional to wavelength
- Signal range and minimum antenna size are directly proportional to wavelength
- Signal quality is directly proportional to frequency

The live video feed should be clear enough and hence its at the 5.8Ghz frequency. Also since it is far from 2.4Ghz there will be minimal overlap between the two signals.

Hence 2.4Ghz gives better range but with lesser bandwidth (amount of data) whereas 5.8Ghz gives can carry more data (higher the frequency more the energy) but has lesser range and is more prone to interferences like mobile towers and electric poles.
