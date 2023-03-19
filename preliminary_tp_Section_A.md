# Section A

---

#### 1 What are the types of motors available for multi-rotors? Explain in detail which one is the most efficient and why with an extensive comparison with the rest.

The two types of motors available for multi-rotors are:

- Brushed

  It has more components as compared to brushless motors due to the presence of commutators and brushes

- Brushless

Brushless motors are more efficient for the following reasons:

- Absence of brushes
  Except for the shaft support, the stator and rotor in a brushless motor are virtually separate entities. This reduces mechanical energy loss due to friction, thus giving longer life
- Better speed control
  Since brushless motors have three input wires as compared to just one in brushed motors (excluding ground), they offer more precise control. For example, sudden motor speed changes can be performed while performing abrupt manoeuvres.
- Outrunner motors (rotor rotating outside over the stator)
  It isn't easy to have an efficient DC outrunner motor because of the brushes, which would need to be somehow mounted on the outer periphery of the motor casing. This would make the design too bulky and prone to damage. Source: https://drones.stackexchange.com/questions/844/are-there-any-brushed-outrunner-motors

---

#### 2 How is the working of a tricopter different from that of a quadcopter? Imagine you are building a UAV for a competition; which one would you prefer? Give a detailed explanation.

A tricopter has three arms, whereas a quadcopter has four. All four quadcopter arms have one motor each; two spin clockwise, and the other two spin counterclockwise. Even the tricopter has one motor per arm, but one of the motors is made moveable on the axis of the arm by connecting it to a servo motor. This is used for the yaw movement, which otherwise would not be possible, like in a quadcopter, wherein the yaw movement can be done by changing the speed of the motors.

For a competition where payload carrying capacity is important, we would prefer a quadcopter over a tricopter as it has more thrust and can be used to carry cameras, medicine etc. But if the competition is racing-oriented or asks for longer flight time, we might go for a tricopter as it is more agile and due to the lesser number of motors it can give better battery life.

---

#### 3 Explain in detail the flight movement shown by drones on the three axes. How do they achieve those? How are these movements achieved on a fixed wing plane?

Flight movement shown by drones on the three axes are:

- Roll

  Movement of a drone about the longitudinal axis. The word "longitude" used here is similar to earth's longitude, which means it's vertical. This movement is required to make the drone go left or right. To go left, both the motors on the right of the quadcopter spin faster than those on the left and vice versa.

- Pitch

  Movement of a drone about a lateral axis. This movement is used to make a drone move forward or backwards. To go forward, both the motors on the back of the quadcopter spin faster than those on the front and vice versa.

- Yaw

  Movement of the drone about the Z axis or the vertical axis. This kind of motion is used to change the heading of the drone. For example, a drone might have a camera mounted on it one side of the drone. When airborne, if the camera has to capture something on the left, the drone itself has to move ninety degrees to the left. This is the yaw movement.

  It is achieved by making one set of diagonal propellers run faster than the rest. The other set of propellors can be made to rotate a bit slower, too, as the balance of the quadcopter would still be maintained.

  This causes yaw because, in a quadcopter, the alternate propellors rotate in similar directions to maintain a net angular momentum of zero; if one set of alternate propellors is spinning clockwise, the other will spin counterclockwise. When one set increases speed, the angular momentum shifts in their favour and yaw occurs.

  It is useful to note here that if instead of alternate propellors if two successive propellors spun in the same direction when the yaw moment is done, if that set is increased in speed, there would be a thrust imbalance, and the quadcopter would roll over. Hence the alternate `cw-ccw-cw-ccw` configuration is opted for.

On a fixed-wing plane, these movements are achieved by the use of servo motors which move parts of the plane's fuselage:

- Roll: Aileron

  Behind each wing at the edge, there will be a movable flap. To make the plane roll, the flaps on either wing will rotate in opposite directions. This causes a change in the lift on both wings and causes roll movement.

- Pitch: Elevator

  The back ends of each horizontal tail surface will be movable in the up-down direction. Both the flaps will move upwards to make the plane pitch up, causing a low-pressure region under the flaps and a high-pressure region above it, and vice versa.

- Yaw: Rudder

  The back edge of the vertical tail fin is movable. Moving it to the right would cause the right yaw and vice versa.

---

#### 4 Explain in detail the working of an ESC and the way it controls a BLDC motor.

An Electronic Speed Controller (ESC) has the following parts:

- Output wires for the motor \* 3
- Power input wires \* 2
- Signal cable \* 1
- \+ 5V cable \* 1

When the ESC is initially switched on with a Lithium Polymer (lipo) battery:

- the Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable

- it waits for a 50Hz pulse width modulated (PWM) signal on the signal wire from either an R/C receiver or the flight controller

An ESC has three main functions:

- Making the Brushless Direct Current (BLDC) rotor rotate

  A BLDC motor is similar to a 3-phase AC motor. Taking the example of an outrunner motor, it consists of a stator which is divided into three similar arms 120 degrees apart with copper windings on them. The spinning part is the rotor which is present over the stator. The rotor has magnets lined up through its circumference.

  When an input signal is received, the ESC starts by providing a positive voltage, say 5V, on one of the motor inputs while keeping the other two inputs at -5V for a very short time, say 100 ms. This causes the copper coil in the arm connected to that wire to get magnetized and thus attract the rotor.
  After 100 ms, it switches the power to another motor input, making the other two motors -5V. The rotor is now attracted to this other arm of the stator. And after another 100 ms has passed, it repeats the process. This subsequent switching of the voltages in the three motor inputs makes the rotor rotate. The rate of switching determines the speed of the motor, and the signal wire controls this rate.

- Improving motor efficiency

  When the motor rotates, due to Faraday's law, a reverse current is generated in the motor cables, especially the ones that are in the Off state. The ESC "listens" to this feedback given by the motor and accordingly adjusts the current to optimize the motor output.

- BEC

  The Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable. Not all ESCs possess BECs.

Apart from these main functions, an ESC also provides functions with some secondary functions dependent on the firmware:

- Quick Braking

  Making the rotor stop suddenly by causing it to suddenly rotate in the opposite direction when the throttle stick is brought to zero. This feature prevents damage to the propellor of a fixed-wing plane while landing because a rotating propellor has both angular and linear momentum. In contrast, a non-rotating propellor has only linear momentum; hence, lesser force acts on it if it hits the ground.

- Safety features

  If the R/C throttle is not at zero when the ESC is powered on, it won't let the motor start unless it's brought to zero first.

---

#### 5 What are the operational frequencies of the electronic components on a quadcopter? Explain why those are chosen specifically and what are their advantages and disadvantages.

As the ESC uses a PWM signal, it operates at 50 Hz. The frequency of the flight controller will depend on the microcontroller chip used.

The radio control for the drone's movement is done at 2.4 Ghz frequency, whereas the video transmission is usually done at 5.8 GHz.

This is because of the following facts:

- Frequency is inversely proportional to wavelength
- Signal range and minimum antenna size are directly proportional to wavelength
- Signal quality is directly proportional to the frequency

The live video feed should be clear enough; hence it's at the 5.8Ghz frequency. Also, since it is far from 2.4 GHz, there will be minimal overlap between the two signals.

Hence 2.4 GHz gives better range but with lesser bandwidth (amount of data), whereas 5.8 GHz can carry more data (higher the frequency, more energy) but has lesser range and is more prone to interferences like mobile towers and electric poles.
