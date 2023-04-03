# aeroMIT_TP_01

# Section A

---

### 1. Read up on all the different components required in the construction of a quadcopter. Prepare a detailed report on the use and working of each of these components. Also for every component, search for commercially available parts and explain their specifications.

#### - Frame

The frame represents the skeleton of the drone. It can be made from various materials such as: wood, aluminium, plastic, or carbon fiber. Out of these carbon fibre is the most popular because it is light weight and durable at the same time and hence can withstand the high stresses and impacts that drones encounter during flights and crashes. The rigidity of a frame is given by the stiffness to weight ratio and this is very high for a CF frame. The downsides of carbon fiber are that unlike plastic, CF is electrically conductive, hence if one has have live wires touching the frame it could cause a short circuit and burn out components. Also, it can block/attenuate radio frequencies due to which antennas should be mounted outside of the frame for optimal signal strength.

The different frame configurations available are:

- H Frames

  In this frame the front and back arms are perpendicular to the middle plate, forming an "H" shape. This leads to a long and roomy body section for installing electronics, cameras, etc

- True-X Frames

  In this frame all the arms intersect at the centre forming an "X" shape, with equal distance between all the motors to the center of the frame. They tend to have a more balanced performance as the motors are contributing equally and hence it is the most commonly used configuration for a quadcopter

- Stretched X Frames

  It's like a True X frame but with the distance on the pitch axis more than the roll axis. In addition to providing the rear propellers with cleaner airflow when in forward or backward motion, this results in the moment of inertia about the pitch axis being higher, which means that it requires more work for an external force like wind to change the pitch of the quad inflight. It also allows a more quickly accelerated response to the pilot's commands on the pitch axis as now it has greater control authority

- Box Frames

  It is similar to an X Frame but looks like a box as there are connecting arms between the motors. This basically creates a tougher frame that is less likely to have broken arms but the extra material creates more drag and weight

- Plus Frame

  This is again similar to the X frame but the drone moves as a plus (+) in the air. This frame is preferred by cinematography drones because there is lesser turbulance as the side motors will always be spinning in clean air

- Deadcat

  The forward two arms are at a larger angle as compared to the rear arms and hence similar to the plus frame, this reduces turbulance on the rear motors due to the forward motors. This makes the drone more stable and hence this configuration is used mostly for cinematographic applications

- Unibody Design

  When compared to a separate arms design, this frame has the benefit of not having any screws but in case of any damage, the entire frame will have to be replaced

There are two variables while choosing a quadcopter frame:

- Number of arms
  A quadcopter will have four arms, a hexacopter will have six arms and so on

- Size
  Frame size refers to the distance between any two diagonal/alternate motors, in millimetres. Generally, a racing drone will have a smaller frame whereas a drone used for videography, search and rescue etc will have a larger frame as it has to carry more load.

Commercially available parts:

- 235mm frames are used for racing drones
- F450, F550 etc are used for drones that require payload capacity (₹749)

---

#### - Landing Gear

These are like the feet of a drone and they exist at it's bottom. They are generally found on heavier drones which are carrying some sort of payload underneath them, for example a camera, and hence can't land on their belly. Racing drones and other small drones like the "DJI Spark" don't have landing gears as their cameras are usually mounted on the top or in front of the drone.

---

#### - Motors

Motors are one of the most important components in a drone as they provide the necessary thrust required for a drone to fly. They are generally mounted at the extreme of the arms of the frame using motor mounts. Quadcopters usually use brushless DC motors (BLDC) which have several specifications such as:

- RPM (KV rating)
- Thrust (unit: grams)
- Efficiency (unit: grams / Watt) It is to be noted that the efficiency of the motor is inversely proportional to it's speed
- maximum voltage
- maximum current
- number of poles: number of magnets on the rotor, a higher number gives more switching speed and vice versa
- number of windings
- resistance: usually < 0.1 Ohm

The rotor has two main components: rotor and stator. The rotor the part that rotates, it carries the magnets whereas the stator is the part which has the copper windings, it generates the electromagnetic field which causes the rotor to move. The reason why the wires wound on a rotor don't short out is that they are enameled, that is, they are coated with a very thin layer of insulation. No-load motors consume very less current when they and hence they don't really produce any torque until a load is attached.

The RPM is determined by the motor's KV rating (velocity constant); it basically gives us an idea about the RPM of the motor. Also, the back EMF generated by a motor is `RPM / KV rating`. For example, a 2000 KV motor will run at 22000 RPM when the input voltage is 10 Volts. Typically larger drones use lower KV motors because they need more thrust in exchange of speed and vice versa. Generally as the KV of the motor increases, it's size decreases as it has to provide lesser torque. In reality, due to losses such as heat losses, the motor will run at a maximum speed of around 80% of `KV * Voltage`.

#### The torque produced by a motor is INVERSELY proportional to its KV rating and hence is also INVERSELY proportional to its RPM since a higher KV rating translates to higher RPM.

### `Mechanical Power = Torque * RPM`

### `KV = RPM = 1 / Thrust = 1 / Torque`

#### `Torque = current = Efficiency = Thrust = 1 / RPM` and because of this the maximum torque of a motor is generated when its nearing stall speed (rpm~0) because at that moment the back emf is minimum, and so the forward current is maximum.

`"=" implies "is proportional to"`

#### This means that a motor can broadly be of two types:

#### - High thrust, High Torque and Low rpm, More payload

#### - Low thrust, Low torque and High rpm and , Less payload

Source: https://things-in-motion.blogspot.com/2018/12/how-to-estimate-torque-of-bldc-pmsm.html

Some motors are sensored which means that they are equipped with hall effect sensors which allow the ESC to sense the position of the rotor. The wires in a motor create a magnetic field which can be measured by the Hall sensors.

The motor naming follows a certain scheme. For example, in an A2212 motor, the first two numbers, 22, represent the width of the stator, that is 22 mm. Whereas the last two numbers, 12, represent the height of the rotor. Typically, motor's with higher volume will have higher torque (higher KV) as they can have more coils in them.`

Mechanical power (Watts) = Torque (Nm) _ Speed (rad/s)
Electrical power (Watts) = Voltage (V) _ Current (A)
Motor Efficiency = Mechanical power / Electrical power

Commercially available BLDC motors:

- A2212 ~900KV to ~2200 KV Outrunner (₹699)
- Turnigy TrackStar 13.5T Sensored 3040KV
- Turnigy AX-2204C 1450KV Outrunner

---

#### - Propellors

These are also an important component of a drone as they convert the rotation of the motors into the actual thrust that provides lift to the drone. On a quadcopter the alternate propellors spin in the opposite direction, that is, two spin clockwise and the rest two spin countner clockwise. This is done as compared to all the propellors spinning in the same direction in order to have net zero angular moment and to prevent unnecessary yaw.

Propellers on most drones are made of plastic, while better models have propellers that are made of carbon fiber. Some models come with propellor guards which can protect the propellers from breakage in situations like the drone hitting a wall or tipping over when landing. Propellors can be made of plastic or Carbon Fibre, with CF giving the advantage of reduced vibrations.

Propellors are usually two or three bladed. Three bladed propellors are usually smaller and can give more lift and provide smoother flights by reducing vibrations as well as noise. On the other hand two bladed propellors give more efficiency as a higher thrust is more efficient compared to higher speed, with the same power.

Propellors have two main characteristics: The length of the blades (diameter) and the geometric pitch, both of which are expressed in inches. The pitch determines the speed whereas the length of the propellor decides how much thrust it can provide. Different combinations of these two specs give us propellors for two broad applications:

- larger propellor
  Higher thrust; gives a lot of lift along with a stable flight but needs more power (current) for each spin. They tend to have higher efficiency

- small propeller
  Lesser thrust but higher speed; less stable flight. Used mainly in racing drones

Most simply put, the pitch of a propeller is how far forward that propeller would move in one revolution. The average pitch for a quadcopter is 4.5 inches. A high pitch like 6 inches will usually result in more overall thrust and top end speed, but less low end torque. This means that the quadcopter will fly faster but will be less responsive, especially when doing maneuvers, as it has lesser torque.

On the other hand a low pitch propellor, eg 3.5 inches, will give less overall thrust and top end speed, but more low-end torque. This means that the quadcopter will be slower, but we can have better control over it: fast direction changes are much easier and more responsive with a lower pitch propeller due to the increased low end torque.

Propellors are by design required to work with motors of a certain KV rating, for example the 6x4.5 propellors are recommended to be run with 800KV - 2500KV motors.

Some commercially available propellor variants:

- 10x4.5 (₹195)
- 6x45

---

#### - Electronic Speed Controller (ESC)

It is an electronic circuit that controls and regulates the speed of an electric motor. It acts like a mediator between the Flight Controller (FC) and the motors; it takes the input from the FC, and translates it to the motor rotation. It may also provide additional features such as a Battery Eliminator Circuit (BEC), dynamic braking, etc.

Connections of an ESC:

- Output wires for the motor \* 3
- Power input wires \* 2
- Signal cable \* 1
- \+ 5V cable and Ground \* 1

When the ESC is initially switched on with a Lithium Polymer (lipo) battery:

- the Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable

- it waits for a 50Hz pulse width modulated (PWM) signal on the signal wire from either an R/C receiver or the flight controller, when it gets a signal it makes the motor run at the speed indicated by the signal

An ESC has three main functions:

- Making the Brushless Direct Current (BLDC) rotor rotate

  The ESC in simple terms generates a sine wave alternating current and provides it to the motor's input cables. The voltages in each of the wires will have a phase difference with respect to each other and that is what makes the motors rotate. If the input signal asks to increase the motor speed, the ESC increases the amplitude of the sine wave. This does increase the motor speed as it's directly proportional to the current but it also increases the back EMF. `i = (Input EMF - Back EMF) / Resistance of the Stator`. The ESC detects this and counteracts it by increasing the frequency of the sine wave.
  Source: https://electronics.stackexchange.com/questions/262106/why-do-brushless-motors-have-a-kv-rating

  ESCs don't convert the DC input to Alternating Current (AC), they just make a waveform that looks like AC. That is not real AC because the polarity doesn't reverse like in true AC.

- Improving motor efficiency

  When the motor rotates, due to Faraday's law, a reverse EMF is generated in the motor cables. The ESC "listens" to this feedback given by the motor and accordingly adjusts the current to optimize the motor output.

- BEC

  The Battery Eliminator Circuit (BEC) steps down the input voltage to 5 Volts and provides the same on the 5V cable. It also helps in reducing the noise (spikes) in the input voltage. Not all ESCs possess BECs.

Some commercially available ESCs:

- REES52 30A ESC (₹500)
- Turnigy Plush 12A 2-4S ESC

---

#### - Battery

The most common type of batteries that are used on a drones are Lithium Polymer (LiPo) batteries. They have a very good weight/power ratio. This means they offer the best combination of power density, energy density and shell life. A battery is a combination of one or more cells. A single celled Lipo cell supplies a nominal voltage of 3.7 Volts(V). When it's fully charged it supplies 4.2 V.

There is no specific minimum voltage for a battery to be called as empty. In the flying community, the common opinion is that going below 3.6 V under no load has chances of damaging the cell. No load state is when the battery is not supplying any current. The voltage output of a battery tends to go lower when it is supplying power, so a 3.6V cell might show ~3 V when under load and that voltage is usually considered the lower dead end. Going below this voltage can cause permanent damage to the cell due to breakdown of its internal components.

Working of Lipo Battery:

A battery is made up of an anode (-ve), cathode (+ve), separator and an electrolyte. The anode and cathode store the lithium. The electrolyte carries positively charged lithium ions from the anode to the cathode and vice versa through the separator. The movement of the lithium ions creates free electrons in the anode which move to the cathode and thus generates electricity. The separator blocks the flow of electrons inside the battery since if it happens there will be an explosion due to high current.

While the battery is charging, the lithium ions move from the cathode to the anode, generating a flow of electrons from the cathode to the anode, that is, the reverse of the discharge process occurs and the battery gets charged.

The specifications of a lipo battery are:

- S rating
  This tells us about the number of cells. For example a 3S cell consists of three cells connected in series and it will have a nominal voltage of 3 \* 3.7 V = 11.1 V

- mili-amp hour (mah)
  This denotes the capacity of the battery. A 5000mah battery will last for an hour if 5 Amps of current is drawn from it, say from one motor. So on a quadcopter, if there are four motors, 20 Amps of power is drawn and hence the battery will last for 15 minutes

- Charging C rating
  It's the cell's maximum discharge current. Say the battery mentioned above has a C rating of 20. This means the maximum current it can provide, without getting damaged is `5A \* 20 = 100 Amps`. A battery is capable of giving higher currents, that is, burst current than the C rating for a short period of time The burst C rating of a motor is usually twice the normal C rating. Higher C rating increases the battery weight and vice versa

- Discharging C rating
  This tells us the maximum charging current. It is recommended to charge batteries at 1C, anything above 2C can damage the battery. So for example, a 5000mah battery can be charged at 5A \* 1C = 5 Amps maximum current. The discharge rating is very much higher than the charge rating by design because a drone needs large amount of current while flying and so the battery should be able to provide it, that is, it's resistance on the discharging direction of the circuit is far less than in the opposite direction

A Lipo battery has two main connections:

- Output wire
  This is the primary connection used to get power out of the battery to power the drone's motors. The wires are usually very much thicker than the charging wire and are terminated commonly by T or XT60 connectors

- Balance Charge Wire
  This is used to charge the battery as well as measure the voltage of the cells. For a 3S battery, it consists of 4 wires: 3 wires for each cell and a common ground

Charging at higher currents (higher C ratings) is more damaging to the battery's cells and is more likely to cause complications like fires and explosions while charging. The opposite is true for charging at lower currents. It is hardly ever recommended to charge at more than 2C, and staying as close as possible to 1C is always recommended for safety and battery longevity.

Lipo batteries have some shortcomings:

- Delicate construction

  Any kind of puncture can lead to an explosion. This can happen if the battery is tightly mounted on the drone and a screw or some other pointed object pierces it. Even while charging it's considered safe to keep the battey within a fire safe "Lipo bag" which is made of fire proof materials

- Battery Swelling

  When a lipo battery is constantly used for long periods, especially when it's overcharged or over discharged, electrolyte decomposition occurs, leading to oxygen gas buildup, which is a fire risk. Since the outer covering of the battery is completely sealed, the gas can't escape, and the battery puffs up.

- Difficult to Dispose

  Disposing a Lipo battery involves initially bringing its voltage close to zero by keeping it immersed in salt water

Commercially available batteries:

- ZopPower Orange 11.1V 30C 2200mAh 3 Cells Lipo Battery (₹1,799)
- Turnigy 4000mAh 3S 30C Lipo Pack

---

#### - Power Distribution Board

The PDB is the hub to which the battery cable is connected from where the current goes to the motors. Usually it is provided with the frame in case of larger quadcopters but in smaller drones people choose to use 4in1 ESCs which is basically four ESCs integrated into one single board, the same size as an flight controller. Using 4in1 ESCs can make the wiring much cleaner but if one of the ESCs on the board gets damaged the entire board is wasted

Commercially available PDBs:

- Invento F450 Quadcopter Frame PCB Board (₹1,050)
- ESC Power Distribution Board for APM/CC3D/MWC Multiwii/KK MultiCopter Quadcopter (₹179)
- SK450 Dead Cat "Defibrillator" Power and A/V Distribution Board and Power Shield

---

#### - Power Module

Since the components of a drone other than the motors usually run at a voltage of 5V, the battery voltage needs so be stepped down. Power Modules do this just like the Battery Eliminator Circuits present in ESCs. They can also have additional features such as vattery voltage telemetry.

Commercially available power modules:

- Robocraze APM Power Module(5.3V/3A) With XT60 Connector
- Power distribution boards with BEC

---

#### - Radio Control (Transmitter and Receiver)

Radio transmitters and receivers are electronic devices which generally use radiowaves, that manipulate electricity resulting in the transmission of useful information through space. The transmitter sends a signal over a frequency to the receiver following a certain protocol.

The protocols vary from one manufacturer to another but the most commonly used frequency is 2.4 Ghz because of the following reasons:

- They are a balance between penetration power through walls (antenna size, dependent on wavelength), and bandwidth (dependent on frequency), latency ie higher data carrying capacity (5.8Ghz)
- FCC regulations make it ISM band: industrial, scientific and medical, no license required to operate on them
- They don’t clash with other license requiring bands such as 1.9 GHz which is used for cellular communications
- The chips used to transmit these signals are cheap

Some of the common protocols available are:
Here is a list of common transmitter protocols:

- ACCST (Frsky)
- DSM & DSM2 (Spektrum)
- AFHDS (Flysky)
- FASST (Futaba)
- CRSF (TBS Crossfire): allows telemetry to be sent

The Radio Control system in a drone consists of two main components: Transmitter and Receiver.

#### - Transmitter

It is held by the pilot and contains two sticks known as joysticks or gimbals. They are used to perform the Throttle, Roll, Pitch and Yaw operations. There are four configurations (modes) to make the sticks perform these actions; the most common one is Mode 2 under which the left stick controls the Throttle (Up-Down) and Yaw (Left-Right) whereas the right stick controls the Pitch (Up-Down) and Roll (Left-Right). The right joystick self-centers in both axes using springs, while the left joystick only self-centers in the yaw axis and the throttle stays wherever it's left to allow for constant throttle.

The mode of a transmitter can usually be changed by manually swapping the left and right gimbals, and/or through software. The other modes are:

- Mode 1
  Left: Pitch and Yaw
  Right: Throttle and Roll
- Mode 3
  Left: Pitch and Roll
  Right: Throttle and Yaw
- Mode 4
  Left: Throttle and Roll
  Right: Pitch and Yaw

Based on how the sensing of the joystick position is done there are two types of joysticks:

- Potentiometer based
  These are cheaper and are mechanical, that is, they sense the position of the joystick by measuring the resistance. These are more prone to wear as there is constant contact within surfaces

- Hall Effect Sensors based
  They are costlier compared to Potentiometer based gimbals but provide longer life as there isn't any physical contact. They work on the Hall Effect principle. There will be a magnet mounted on the joystick whose movements can be measured by the sensor due to the changing magnetic field

One of the main specifications of a transmitter is the **number of channels** it can have. Each type of data input requires a channel. For example, a basic transmitter with just the throttle, roll pitch and yaw controls will have just four channels. A radio control should have minimum five channe;s for piloting a drone, 4 channels for the joysticks and the fifth channel for arming the drone. It can have additional channels for other features such as changing flight modes.

The process of connecting a transmitter to a receiver is called "Binding". Each transmitter has a unique ID, when binding with a receiver, the receiver saves that unique ID and can accepts only data from the unique transmitter. this avoids picking another transmitter signal and dramatically increase interference immunity and safety. In order to be put into bind mode, all receivers have a bind pin which need to be connected to the ground pin using a "bind plug". Once this is done the receiver's indicator light starts blinking repeatedly, after being bound the blink rate reduces and after a restart, when it connects with the transmitter, the light will be solid coloured, usually red.

Most transmitters ask for failsafe settings to be setup which are stored in the receiver. They are basically the inputs that the receiver will use in case of loss of signal. As part of the failsafe, the throttle is usually set to zero because the drone coming to the ground is usually the best case scenario.

A transmitter is usually powered by rechargeable AA batteries but some transmitters also use Lipo/Lion batteries for power input

#### - Receiver

It is mounted on the drone and will usually have two antennas. It's a crucial component that receives the radio signals transmitted by the transmitter. It is usually supplied along with the transmitter in order to prevent protocol mismatch. It is powered from the power module or from the flight controller. The received signals can be sent to the F/C through different protocols such as:

- Pulse Width Modulation (PWM)

  Data is relayed by varying the pulse width. Each RC channel has its own cable. Hence if we want six channels, there will be six cables from the receiver to the FC. The value of each channel is represented as a 1 millisecond to 2ms "ON" signal, which repeats every 20 ms (50 Hz frequency). The amount of time it is "ON" is the value for that channel.

- Pulse Position Modulation (PPM)

  PPM operates similarly to PWM, with the difference being that all the channels are carried on a single cable. This is accomplished by lining up several PWM signals back to back; each channel is sent successively, followed by a delay. When all the channels have been sent, it loops back to channel one. Normal PWM operates at 50Hz frequency, which means that each update takes 20 milliseconds. So if each channel takes up to 2ms, a maximum of 10 channels can be fit within that 20ms. Hence PPM allows there to be just one cable from the R/C receiver to the Flight Controller for all the control channels.

- sBus & iBus

  While PWM and PPM are analog protocols, sBus and iBus are serial a.k.a digital protocols and they too can carry data over a single cable. They have been designed by companies named Flysky and Futaba and have many advantages compared to PPM such as better latency, up to 16 channels (while PPM has maximum 20), error and parity checking etc. One disadvantage is that they don't provide telemetry

The connections on the receivers include three rows of pins; signal, positive and ground. The positive and ground wire power the receiver from the FC. Each pin column stands for a particular channel. In case of PPM connection, a single wire is connected to the one of the channels, usually channel one.

The antennas of the receiver are suggested to be kept perpendicular to each other in order to have better connectivity with the transmitter. Best case is when the antenna of both the transmitter and the receiver are parallel to each other.

Commercially available R/C controllers:

- Flysky Fs-I6 2.4G 6Ch Afhds Rc Transmitter Controller with Receiver (₹5,599)
- Robocraze Flysky CT6B Remote 6Ch Transmitter and Receiver
- Turnigy 9X 9Ch Mode 2 Transmitter w/ Module & iA8 Receiver (AFHDS 2A system)

---

#### - Flight Controller (FC)

It is one of the most important components in a drone since it’s responsible for stabilizing the aircraft (PID), ensuring precise flight manoeuvres (flight modes), and providing data to the pilot (telemetry). It is a hardware hub that most of the drone's components will attach to and hence it's like the brain of the drone. Over time flight controllers have shrunken in size and also added more features such as having ESCs embedded within them. Some of the features an FC can provide apart from making the drone fly are:

- Automatically leveling the drone
- Loitering where the drone locks in place
- Autonomous missions
- Return to home functionality
- Black box

A Flight controller is essentially a circuit board with a microcontroller, equipped with sensors that detect the drone’s movements and user commands. With this information, the FC adjusts the speed of the motors to move the drone in the desired direction. Most FCs have four main (Microelectromechanical systems (MEMS)) sensors which integrate mechanical/electrical components within 1 to 100 micro metres. They are built with microscopic motors that respond to the surrounding environment by moving in response to g-force. As they shift position, they send electrical signals which give precise information about the position, velocity, etc. Some common specifications about the sensors are startup time, maximum operating temperature, communications protocols supported, etc. The common senors are:

- Gyroscope

  It provides the extent and rate of rotation in space (roll, pitch and yaw)
  Gyroscopes also measure the rate of rotation, allowing a drone to remain stable when hovering or when in flight.
  Eg. L3G4200D

- Accelerometer

  It is used to measure linear dynamic acceleration. Static acceleration is the constant force acting on a body, like gravity or friction wheras dynamic acceleration are non-uniform. The working of an accelerometer can be understood by imagining a box in outer space with a ball present at its centre. If the box is abruptly moved in any direction, it will hit one of the walls of the box. Similarly in a MEMS accelerometer has fixed plates built on top of a silicon wafer which are analogous to the box. Now the ball inthe box is a suspended structure which also has plates attached to it. Any kind of deflection causes a change in capacitance between fixed plates and plates attached to this suspended structure. This change in capacitance is proportional to the acceleration along that axis. The sensor processes this change in capacitance and converts it into an analog output voltage.
  Specifications:

  - number of axis
  - resolution measured in bits
  - maximum Gs, for example, 1G is `9.8m/s^2`, 2Gs is `9.8m/s^2` \* 2 = `19.6 m/s^2` and so on
    Eg. ADXL345

- Magnetometer

  It helps a drone orient itself in relation to the magnetic north. The sensor converts any incident magnetic field in the sensitive axis directions to a differential voltage output and Communicates via I2C. The sensitive axis is basically a line of resistive strip elements which are magnetoresistive sensors made of a nickel-iron thin-film. In the presence of a magnetic field, a change in the bridge resistive elements causes a corresponding change in voltage across the bridge outputs. Because the output is only proportional to the magnetic field component along its axis, multiple sensors are are placed orthogonally, on every axis to allow accurate measurement of magnetic field in any orientation.
  Eg. MC5833L

- Barometer

  It measures the altitude of the drone by measuring the drop in air pressure as a drone flies upwards and vice versa

- Self Test

  Most sensors have a dedicated contact for this function which is used to ensure that the senor is not faulty. Before this feature came into existance, testing a sensor, say for example an accelerometer involved manually tilting the PCB or the FC it was mounted on but now the sensors can themselves simulate the movement and return a faulty or not faulty signal. To perform the simulation an electrostatic test force is applied to the mechanical sensing element which causes the moving part to move away from its original position, emulating a definite input acceleration. The output from this simulated motion is compared with the measurements in the "normal" mode. If the delta value is absurdly high then the sensor is said to be faulty.

The Gyroscope and Accelerometer are housed within a unit called the Inertial Measurement Unit (IMU) whereas the Magnetometer and Barometer is separate.

The components connected to a FC are:

- R/C Receiver via protocols like PPM, iBus, sBus etc
- Power module
- ESCs via PWM protocol
- Telemetry Module
- Additional sensors such as GPS Module, Lidar sensor
- Indicator lights

The three main types of connectors on a flight controller are:

- Plastic JST connectors
- Solder pads
- Through holes

Devices are usually connected to the FC using JST connectors

The software running on a FC is called firmware and there are different firmwares available for controlling a FC:

- ArduPilot
- PX4
- iNav
- BetaFlight
- KISS

Some things to consider while buying an FC:

- Microcontroller Unit, for example STM32F745, F4, F7, H7
- IMU: MPU6000
- Barometer: BMP280
- Power I/O Specifications
- Available connections apart from R/C in and ESC out like UART, SPI etc
- Supported Firmwares

An FC can't be autonomous by itself unless it's very high end. So in order to make an autonomous drone, a Companion Computer is attached to the FC. This can be any small computer, a Raspberry Pi for example. It is usually connected to the FC by serial protocols like UART. The Companion Computer would have code, say python code which will take the data given by the FC and accordingly instruct the FC on hoe to move the drone. In this situation the FC acts like an API (Application Package Interface) since it makes very easy for the Companion Computer to move the drone.

Commercially available Flight Controllers:

- APM Ardupilot: Software used: Mission Planner
- Pixhawk: Mission Planner
- Naze32: Betaflight
- DJI A2, DJI N3
- T-Motor Mini F7 HD Flight Controller (MPU6000)
- SpeedyBee F745 35A AIO Flight Controller

The APM flight controller uses an 8bit Atmega2560 microcontroller, uses an I2c bus and runs at 16Mhz. On the other hand, Pixhawk uses an ARM Cortex 32bit STM32 F4 running at 168mhz on a much faster SPI bus, has the faster CAN bus and has a backup processor STM32 F1 chip that runs at 72 Mhz. It also has a microSD card for flight logs. A faster processor means that it can compute better PID loop times thus can keep the drone more stable. Updates to APM's firmware, Arducopter, are no longer made for the APM as there is not enough memory or power in the APM to support them, hence Pixhawk is the preferred option.

---

#### - GPS sensor

Global Positioning System is a satellite-based navigation and location system which can be used by anyone possessing a GPS sensor. GPS is one of multiple other systems such as GLONASS, BeiDou, Galileo etc which have been made my different countries. Usually GPS sensors support all these different systems for redundancy. Most GPS senors include other sensors like the magnetometer and barometer; the barometer may not be as accurate as a real one since the height obtained via GPS may not be that accurate. They communicate with the satellites using the NMEA protocol. It is used for transferring data between marine-related electronics such as GPS receivers, autopilots, and chart plotters. Typical NMEA data includes latitude, longitude, time, and satellite status.

GPS works using the concept of triangulation. Every satellite in the network continuously transmits time-stamped signals which can be received by GPS receivers on the ground. These signals are sent as radio waves that travel near the speed of light and so multiplying the time lag between two consecutive signals with the speed of light gives the distance of the satellite from the drone. Using the data from multiple satellites (three at minimum), the accurate location of any GPS receiver on Earth can be deduced. Triangulation can be understood by imagining three intersecting circles with the drone being at the common point.

GPS senors usualy communicate with the FC via the UART protocol. They use Ceramic or patch antennas which look like a square brick. Due to their small size they can be directly integrated on PCBs but the downside is that they have limited range. By measuring the change in drone's position per unit time, the velocity of the drone can also be calculated. Because these antennas are directional the GPS sensor can perform best when in open space and finds it difficult to find a GPS Lock when indoors or when the drone is flying under trees or inside a building.

Specifications of a GPS sensor:

- Microprocessor
  UBLOX is the most commonly used MCU
- Cold start time
  It is the time required for a the sensor to turn on and get a satellite lock or a 3D fix. Its usually above 30 seconds
- Hot start time
  It is the time taken for the sensor to give the GPS coordinates when it's powered on
- Update rate
  How fast the sensor can provide measurements

Using a GPS sensor increases the abilities of a drone by adding features like:

- Hover
- Altitude Hold
- Autonomous Flights
- Return To Home
  GPS sensor on the transmitter side can aid in RTH being available even if the pilot is in motion
- GeoFencing
  Restrict drone movement within a perimeter
- Location Data Reporting

Total control shouldn't be given to a GPS sensor as if it ever suddenly changes the coordinates due to some malfunction, for example when in Hover mode, the drone might go out of control and zoom to those new, probably fake coordinates which might cause a crash. Same thing can happen if the barometer on the GPS module fails. Hence it's benefitial to have the same senors on the GPS module as well as the FC for redundancy purposes.

Commercially available GPS modules:

- Robotbanao NEO-6M GPS Module With EEPROM-Built-in Active GPS Antenna (₹633)
- Matek GPS and Compass Module (M9N-F4-3100)

---

#### - Antennas

The radiation pattern of an antenna is a doughnut shape, it has the strongest signal on the side and the worst signal on both ends. Due to this, the alignment of transmitter and receiver antennas affects the range tremendously. The best range is obtained when two antennas are parallel (as if they are the opposite sides of a square) and the worst case is when their ends are in the same line.

There are different types of antennas based on polarization. Polarization refers to how the radio waves travel in space. There are two main types:

- Linear

  The signal is send in one plane only. Under this there are two possible polarizations: Vertical and Horizontal. Vertical and horizontal refers to the plane in which the wave is travelling. In theory, a receiving antenna with horizontal polarization (will be horizontal) would not "see" a signal from one with vertical polarization and vice versa, even if the two are operating at the same frequency. The closer their alignment, the more signal is captured, with maximum energy transfer taking place when the polarizations are matched.

  A horizontally polarized antenna will perform better when mounted near a ceiling, whereas a vertically polarized antenna will perform better when mounted near a side wall because in these cases, there will be maximum reflection of the signals from the walls. Wireless systems such as Wifi will typically use linear antennas since the devices are stationary on the ground so its easy to ensure the antennas are always aligned. When using linear antennas on board a drone in the sky, one will get lesser overlap, resulting in the signal breaking down.

- Circular

  These antennas are the first choice for FPV (First Person View) transmission systems. Their working is more complicated than linear polarization as in this mode, the polarization represented by the E-field vector rotates as the signal propagates. The signal isn't sent like a sine wave but instead it is like a spiral. It is generated by having two orthongal linear polarized waves 90 degrees out of phase. While looking from the transmitter end, the signal can rotate right to left, that is, anticlockwise which is called Right Hand Circular Polarized (RHCP) and when the signal rotates left to right, it's Left Hand Circular Polarized (LHCP). RHCP antennas work only with RHCP antennas and not with LHCP antennas and vice versa.

  Another advantage of circular polarization is that a RHCP wave will reflect off a surface and become LHCP, and vice versa. This is advantageous because an antenna designed to receive RHCP waves will have some immunity to the signal-fading effects of reflected wave interfering with the desired wave. These are some of the reasons GPS signals from satellites are also RHCP.

  In general linear polarization can provide extra range as all the energy is focused on a single plane rather that being dispersed into a cylindrical pattern. However in order to get good reception with linear polarized antennas we need to ensure that both antennas are aligned to ensure the radiation pattern has maximum overlap.

  If a linear polarized antenna is used with a circular polarized antenna, there will be a 50% loss in signal because circular polarized waves are two linear polarized waves with a 90 degree phase difference.

Types of antennas based on polarization:

- Linear

  - Monopole

    They are the simplest antennas which consist of just a piece of wire which is the active element

  - Dipole

    They have a T shaped divider with two monopole antennas on each side. Its two poles maintain different polarity so it takes half peak voltage, but the same power, to produce a similar signal strength as that of a monopole antenna. When it's used as a receiver, one end can take the positive end on the wave and the other end takes the negative, thus getting a better signal strength

- Circular Polarized

  - Cloverleaf

    These are omnidirectional antennas, that is, they provide 360 degrees of consistent coverage. They are either three or four lobed (skew-planar). "Lobe" refers to the number of bent wires on the antenna, which are soldered together in the middle. The cloverleaf is most often used for the drone video transmitter whereas the skew-planar antenna is used on the receiver

  - Pagoda

    These are also omnidirectional but they are more durable when compared to the cloverleaf antennas and they also provide better range. They were designed by Maarten Baert and in construction, they are two circular plates parallel to each other. The plates are basically PCBs with a circular design on them

  - Patch

    They are directional antennas which look like a square plate. They have longer range when the receiver antenna is parallel to the transmitter and hence they are generally used as the receiver end as there the antenna can be kept pointed towards the drone. Diversity recievers tend to use a combination of omnidirectional and patch antennas

  - Helical

    These antennas are also directional and look like a spring. The number of turns of coil present determines the gain (strength) of the antenna. Helical antennas with just one or two turns have performance comparable to a typical patch antenna. However, adding more turns can highly improve the range

Antenna Tracking Gimbals can be used to continuously point directional antennas towards the drone. The tracker can obtain the drone's location with the telemetry module.

Types of antenna connectors:

- Coaxial RF connector

  - SMA (SubMiniature Version A)
    The SMA male has a centre pin which screws over the SMA female, which has a hole in the centre

  - RP-SMA (Reverse Polarity SMA)
    It is the exact opposite of SMA; the RP-SMA male has a hole in the centre which screws over the SMA female, which has centre pin

- U.FL

  They are very small, comparatively fragile connectors which have a male end which plugs into the female end. The female end is usually soldered on to a PCB. They are used to connect small antennas, for example the transmitter's antenna to it's PCB

---

#### - Telemetry Module

A telemetry module uses radio waves to transmit telemetry, that is, the readings of sensors on the drone to the Ground Station System (GCS), on which they can be displayed or recorded. It transmits important information like the battery voltage, drone's speed, altitude, heading, the temperature of onboard components, amount of fertilizer/pesticide left in case of agricultural drones etc. It can also be used to transmit live video from the drone. Some telemetry modules can transmit the as well as receive commands to pass on to the flight controller. For example, the APM or Pixhawk flight controllers can be given commands from the "Mission Planner" application running on a ground station. A drone can fly without a telemetry module with only the R/C uplink but doing so is risky because the pilot would have no idea about the status of the components on the drone. It can also be used to command the drone, meaning one wouldn’t actually need an RC controller to fly the drone but due to the lack of joysticks on a laptop, telemetry is mostly used for receiving data back from the drone.

The telemetry system consists of a transmitter and receiver. The transmitter is present on the drone and connects to the FC usually through the UART protocol. The receiver is connected to a laptop with the GCS software running. The radio transmission is done at 413 Mhz in Europe and 915 Mhz in North America. Bluetooth can also be used but it's avoided due to it's low range.

MAVLink is the serial protocol most commonly used to send data and commands between vehicles and ground stations. MAVLink messages can be sent over almost any serial connection and does not depend upon the underlying technology (wifi, 900mhz radio etc). The messages are not guaranteed to be delivered which means ground stations or companion computers must often check the state of the vehicle to determine if a command has been executed.

Commercially available Telemetry Modules:

- Generic 3DR Radio Telemetry 915MHZ Module For APM
- FrSky XJT EU LBT 2.4Ghz JR Module with Telemetry

---

#### - Ground Station Software (GCS)

Ground Station Software is typically a software application, running on a ground-based computer, that communicates with the drone via the wireless telemetry module. The GCS can track the drone’s position on a map, determine how fast the drone is moving, see battery voltage levels, etc. As it displays real-time data on the drone's performance and position and can serve as a "virtual cockpit". It can also be used to control the drone in flight, uploading new mission commands, setting parameters, etc. For example, it can be used to set a new flight plan.

Examples of GCS software are Mission Planner, MAVProxy, QGroundControl, etc. They are also available for smartphone: Tower (DroidPlanner 3), MAVPilot, etc.

---

#### - FPV System

FPV stands for First Person View and it refers to getting a live video feed from a camera mounted on a drone. These systems can be classified into digital and analog systems. In practical terms, digital FPV systems provide very much higher video quality but are more expensive, while analog FPV systems provide lower resolution video at a lower cost. Moreover, the former is less susceptible to interference and has superior range capabilities. This system has three main components:

#### - Camera

FPV cameras have three main connections: video signal, voltage input and ground, all three of which are usually directly connected to the video transmitter. Apart from these, there might be other optional connections like:

- UART interface for connecting the camera to the FC to get telemetry for OSD (On Screen Display)
- OSD settings for plugging in the joystick for changing camera settings
- VBAT for monitoring battery voltage to overlay on the OSD

Specifications of an FPV camera:

- NTSC (National Television System Committee) or PAL (Phase Alternating Line)

Both of these are analogue colour television standards. The main difference between NTSC and PAL is in resolution and frame rate. Resolution refers to the number of pixels width by height. For example, 200 x 100 refers to a 200 by 100 pixel screen. On the other hand, frame rate refers to the number of frames (images) per second in a video. PAL offers slightly better resolution (720 x 576 @ 25fps), while NTSC allows higher frame rate (720 x 480 @ 30fps). This means that PAL gives a better resolution whereas NTSC gives more fluid footage. Conventionally, NTSC is used in North America, Japan and South Korea while PAL is used in most of Europe, Australia and large parts of Africa and Asia.

- CCD (Charge Coupled Device) or CMOS (Complementary Metal–oxide–semiconductor )

CCD is an older technology and hence produces somewhat lower resolution and less detailed pictures than CMOS. This is because the image is more "raw" as compared to CMOS as there isn't much processing happening. The image amplification is done at once on the entire image using a single Charge Amplifier, unlike CMOS sensors where MOSFETs are used to do local amplification.

Lesser jello effect in the footage is it's advantage due to it possessing a global shutter. Jello refers to the image being blurry at times and global shutter refers to a shutter that opens and closer the entire aperture at once. In contrast to this, a CMOS sensor's rolling shutter doesn't open the camera's aperture in one go but does it part by part, much like a rolling shutter door. The image is captured row by row when exposed to light. Due to this, CCD cameras are preferred for racing drones which have higher vibrations as compared to payload carrying drones.

CMOS cameras generally have lower latency, higher resolution and sharper image in exchange for heavier digital noise. The increased noise is due to amplification happening on multiple pixels at once. Low light cameras tend to use large CMOS sensors because of higher resolution and individual pixel amplification. But because of their rolling shutter, they are more susceptible to imaging artifacts such as motion blur and the jello effect. Hence, CMOS cameras are mostly preferred for taking static pictures rather than video.

- Voltage input

Most FPV cameras these days support wide range of input voltage; 5V to 36V. This allows one to power them either from a regulated power source or directly from a LiPo battery (2S-8S), but generally the camera is powered from a 5V output from the video transmitter

- Aspect Ratio

This refers to the physical width to height ratio of the screen. There are two aspect ratios to choose from in FPV cameras; `4:3` and `16:9`. `4:3` is more square, box-like while `16:9` is wider like a TV. Which camera to go with depends on the screen or the FPV goggles since if a `4:3` camera is used with goggles having a `16:9` aspect ratio, the image will appear stretched, and vice versa.

CMOS sensors have a native aspect ratio of `16:9`, while CCD cameras are `4:3` by default. Some CMOS cameras the user to choose between `16:9` and `4:3` in the camera settings, but this might result in a smaller field of view, as it involves chopping off the sides or top edges from the image depending on whether it is a `16:9` to `4:3` conversion or `4:3` to `16:9` conversion respectively.

- Camera Size

  FPV camera sizes are determined by their width; the distance between the two side mounting holes. Larger sizes allow for a larger sensor which increases image quality. The common sizes are:

  - Standard a.k.a full size (28mm)
  - Mini (21mm)
  - Micro (19mm)
  - Nano (14mm)

- Sensor Size

Sensor size is measured diagonally and the two main sensor sizes are 1/1.8 inch and 1/3 inch. This parameter affects low light performance and dynamic range. A camera with larger sensor (larger aperture) will have better low light performance because it can absorb more light due to the increased area. Larger sensors can also offer a larger FOV.

- Field of View (FOV)

The Field of View of a camera is the horizontal anglular view that a camera can provide, it is measured in degrees. With a smaller FOV, the image is more zoomed in and one can see things more clearly. Wider FOV allows one to see more of the environment which might be preferred for racing or flying close to buildings. A simple example is the wide angle camera present in smartphones; it has a wider FOV than the main camera. This parameter depends on:

- Focal length
  A shorter focal length gives a wider FOV, but lesser zoom and vice versa

- Sensor size
  A larger sensor size can also increase the FOV

- Aspect ratio
  If the camera is primarily `16:9`, switching it to `4:3` will chop some amount of side off reducing the FOV

Too large FOV can cause the "fish eye" effect; the objects in the middle will appear smaller and further away than they really are, while the edges of the image will appear curved and distorted.

- Focal length

This refers to the distance of the lens' aperture from its focus. The focus of a converging lens is that point where rays originating from infinity tend to converge on. A higher focal length allows more light in, in exchange for a reduced FOV and vice versa.

Mini and Standard size cameras normally have lenses with 12mm diameter threads which are known as M12 lenses. Similarly smaller lenses such as the M8 are used in some cameras in order to make them smaller and lighter. M12 cameras let more light in and are generally preferred over M8 cameras.

- On Screen Display (OSD)
  OSD overlays useful information, such as the battery voltage, drone's speed, etc over the video, thus providing another form of telemetry, which can be viewed on the screen or through the FPV goggles.

---

#### - Transmitter and Receiver

The transmitter (VTX) is present on the drone and it transmits the video feed from the drone to the receiver (VRX) present on the ground using radio waves. The most commonly used frequency is 5.8 GHz as it provides higher bandwith compared to 2.4 GHz and also reduces interference between the video transmitter and the R/C receiver, as they are present very close to each other on the drone. Circular polarized antennas are commonly used for the VTX and VRX communication. Most analog video receivers utilize the RTC6715 IC whereas the transmitters use the RTC6705 IC. The RTC6715 IC is one of the few ICs that can be controlled via SPI (serial programming interface).

The connections on the VTX include wires to the camera and the power input from the battery. The receiver is connected to the screen or sometimes it's present within the screen or FPV goggles.

---

#### - Screen and FPV Goggles

Specifications for FPV goggles and the screen are more or less similar with the goggles having certain additional specifications such as FOV. Either one or both these instruments can be used by the pilot while flying the drone. Some pilots choose to have both setups with different types of antennas on each for redundancy purposes.

- Resolution
- Aspect Ratio

- FOV
  FOV is the most common measurement in FPV goggles as it tells us how immersive the flying experience will be. If it's not specified whether the FOV is horizontal or diagonal, it’s usually assumed to be diagonal FOV

- Built-in video receiver
  Some FPV goggles come with an integrated video receiver, which can be convenient because they’ll work right out of the box, eliminating the need for additional receiver purchases. But the limitation will be that the receiver within the goggles can't be changed, once purchased

- Digital Video Recorder (DVR)
  DVR, or digital video recorder, is a feature that allows one to record the video feed received by the VRX on to a micro SD card

- Diversity
  A diversity system includes two video receivers within a single module, automatically selecting the receiver with the stronger signal to maintain the best possible signal. On the transmitting side, the VTX will have two antennas oriented perpendicular to each other and on the receiving side different types of antennas can be used, for example, an omnidirectional antenna and a patch antenna. More advanced diversity systems merge the two signals into one, minimizing breakup and static interference; this is referred to as "true diversity".

---

#### - Gimbal

A drone gimbal is a support system that carries a camera and allows it to remain at the instructed angle, regardless of the drone's motion, while it maintains the direction of shooting. It cancels unnecessary motion in the three axes of movement roll, pitch, and yaw. To do so it has three motors or servoes which have control over each of these axes. More advanced gimbals come with an anti-vibration feature that reduces camera shake, so that one can get more stable videos. Gimbals can be controlled by the FC or by a computer within them. The IMU's readings are used to find out the drone's orientation which is then used to keep the gimbal level. For example, if the drone is pitching forward, when seen from the left of the drone, the servo motor responsible for the pitch axis will be moved clockwise to an angle equivalent to the drone's pitch, so as to keep the payload level with respect to the ground.

There are generally two types of gimbals; 2-axis and 3-axis gimbals. They differ in the number of axes they can stabilize. The 2-axis gimbal stabilizes on 2 axes: pitch and roll whereas the 3-axis gimbal provides stabilization on the yaw axis too. With this extra level of stabilization, 3-axis gimbals are great for shooting videos as the additional yaw stabilization reduces horizontal movement giving professional results. 2-axis gimbals can suffice if the primary use of the camera is for taking photos. In addition, 2-axis gimbals are lighter, consume less battery power, and are less expensive than 3-axis models.

---

#### - Lidar sensor

They are used to perform Time of Flight (ToF) distance mapping and 3D imaging. The sensors emit a very short infrared light pulse and each pixel of the camera sensor measures the return time. Using this information, the distance between the senor and the obstacle can be calculated. Appications: object scanning, measuring distance, indoor navigation, obstacle avoidance, gesture recognition, tracking a moving vehicle, etc.

---

#### Pending:

- Cameras: working of CCD and CMOS cameras
- ESC working
- PID
- Q 2

---

### 2. In order to design a autonomous quadcopter, which components would you choose? Explain with exact model names and your reasons for choosing the same (cost and performance must be included in the report).
