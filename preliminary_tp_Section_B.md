# Section B

---

#### 1 What is a PID loop and explain in detail how and why is it used in drones?

A Proportional Integral Derivative controller is a critical drone component that provides stabilization and autonomous control. Based on the inputs from sensors like the Inertial Measurement Unit (IMU) and barometer, it tweaks the speed of the motors to obtain the desirable state (a defined setpoint); for example, to hover in place or move as directed by the pilot. The IMU contains the accelerometer, gyroscope and magnetometer.

The PID Loop refers to the loop constructed by providing the feedback given by the motors to the PID controller and the sensors' input. The PID controller continuously calculates an error value e(t) as the difference between the desired setpoint SP and a measured process variable PV (sensor data) and applies a correction based on proportional, integral, and derivative settings set by the pilot. The controller then uses this correction and attempts to minimize the error over time by adjustment of a control variable u(t) which in the case of drones is the speed of the motors.

Various factors related to the drone's movement, such as its responsiveness to commands, time taken to stop to a hover, etc., can be tweaked by setting the P, I and D parameters in the flight controller's settings.

A brief overview of the P, I and D settings:

- P gain determines how hard the flight controller works to correct an error; it represents responsiveness. A high P value makes the drone snappy, whereas a low value would make it sloppy.

- I gain determines how hard the flight controller works to hold the drone's attitude against external forces, such as wind and off-centred CG. It is like the drone's stiffness setting

- D gain dampens the effects of the P gain and reduces the overshoots. Since the derivative term measures the rate of change of error, increasing it can help detect when the P gain is reducing error way too fast and thus soften it. The drone can suffer from bounce-backs if the D value is too low.
  For example, stopping the drone suddenly after moving forward for a while could cause it to pitch backwards. This would be because the P gain would want the drone to stop and hover, and hence it would increase the speed of the forward rotors and vice versa. But due to low D gain, the forward motors might increase in speed too much, causing backward pitch.

---

#### 2 What are the different types of modulation techniques available for use in drones ? Explain these protocols in detail.

The two modulation techniques available for drones are Pulse Width modulation and Pulse Position modulation.

Devices like Servos and Electronic Speed Controllers use PWM as signal input, whereas R/C transmitters, receivers and Flight Controllers are usually capable of handling both PPM and PWM signals.

- Pulse Width Modulation (PWM)

  It is a technique used to relay data through varying pulse width. In PPM, the analogue sample values determine the position of a narrow pulse relative to the clocking time.

  In PWM, each RC channel has its own cable. The reason for having multiple R/C channels in an aircraft is to have more freedom. For example, in a fixed-wing plane, one channel might go to the throttle motor, whereas two others will go to the ailerons, another two to the elevators and another one to the rudder. Hence if we want six channels, we must wire all six cables along with the power and ground. The value of each channel is represented as a 1 millisecond to 2ms "ON" signal, which repeats every 20 ms (50 Hz frequency). The amount of time it is "ON" is the value for that channel.

- Pulse Position Modulation (PPM)

  PPM operates similarly to PWM, with the difference being that all the channels are carried on a single cable. This is accomplished by lining up several PWM signals back to back; each channel is sent successively, followed by a delay. When all the channels have been sent, it loops back to channel one. Normal PWM operates at 50Hz frequency, which means that each update takes 20 milliseconds. So if each channel takes up to 2ms, a maximum of 10 channels can be fit within that 20ms. Hence PPM allows there to be just one cable from the R/C receiver to the Flight Controller for all the control channels.

---

#### 3 Explain the working of UART, I2C and SPI communication protocols. Give a comparison between them in detail.

UART, I2C and SPI are all examples of Serial communication protocols. Serial communication sends data one bit at a time, sequentially, over a communication channel instead of parallel communication, where several bits are sent as a whole on a link with several parallel channels.

- UART (Universal Asynchronous Receiver-Transmitter)

  This protocol uses two wires known as Tx (transmit) and Rx (Receive) for both components to communicate and is one of the most minimalist serial protocols. Since UART is an asynchronous protocol, it doesn't have a clock that regulates data transmission speed. As an alternative, it utilizes the "baud rate" for timing when a bit is to be transmitted. The usual baud rate used for UART is 9600 baud, meaning a transmission rate of 9600 bits per second. At this speed, one bit takes about 100 microseconds to be transmitted.

  To let the receiver know when data is about to be transferred, a "Start Bit" (low signal) and then to let the receiver know that the last bit (most significant bit) has been sent, a "Stop Bit" (high signal) is sent.

  Specs:

  Minimum number of wires: 1

  Duplex: Full Duplex

  Number of masters and slaves: One each

  An example of a device where UART is used is the HC05 Bluetooth module. To change its settings, it's connected to a computer via a UART to USB serial converter. The Tx of the HC05 is connected to the Rx of the converter and vice versa.

- SPI (Serial Peripheral Interface)

  SPI is another serial protocol used for faster data rates of about 20Mbps. It has lower power consumption than other synchronous protocols like I2C.

  It uses a total of four wires, namely SCK (Serial Clock Line), MISO (Master In Slave Out), MOSI (Master Out Slave In), and SS/CS (Slave/Chip Select). Unlike UART, SPI uses a master-to-slave format to control multiple slave devices with only one master. The MISO wire acts like the Tx from the slave in UART, whereas the MOSI is like the Tx from the master to the slave; that is, it transfers data from the master device to the slave device.

  SCK is clock output from the master and is used for synchronization because, unlike UART, SPI is a synchronous protocol. The SS wire is used to select the slave to be controlled out of the multiple slave devices that may be connected.

  Specs:

  Minimum number of wires: 3

  Duplex: Full Duplex

  Number of masters and slaves: Single master, one or more slaves

- I2C (Inter-IC bus)

  I2C is another synchronous serial protocol like SPI, but with several advantages over it. These include having multiple masters and slaves, simple addressing (no need for Chip Select), operating with various voltages, and using only two wires.

  The two pins in an I2C protocol are the SDA (Serial Data Line) which transmits and receives data, and the SCL (Serial Clock Line) pin, which functions as a clock. Since there is a single wire to transmit and receive data, I2C is Half Duplex.

  The working of I2C is more complicated than UART. When the master wishes to read or send data to the slave, it starts with the master sending a start bit (low signal) from its SDA pin, followed by a seven-bit address that selects the slave and one bit for selecting read or write. The slave responds with an ACK (acknowledgement) bit and listens for incoming transmission. Now in case the master wishes to read data, it selects the register of the slave it wishes to access and sends eight bits specifying it. The slave responds by sending the requested data, after which an Acknowledge bit is sent to the master, following which the master ends with a stop bit (high signal).

  Specs:

  Minimum number of wires: 2

  Duplex: Half Duplex

  Number of masters and slaves: Multiple masters and slaves

---

#### 4 What is a telemetry module and why is it used in a drone? Is it possible for a drone to fly without a telemetry module?

A telemetry module uses radio waves to transmit telemetry, that is, the readings of sensors on the drone to a device on the ground, on which they can be displayed or recorded. It usually transmits important information like the battery voltage, the temperature of onboard components, amount of fertilizer/pesticide left in case of agricultural drones etc.

Some telemetry modules can transmit the information and receive commands to pass on to the flight controller. For example, the APM or Pixhawk flight controllers can be given commands from the "Mission Planner" application running on a ground station.

Telemetry can also be provided on the on-screen display (OSD) by overlaying the information over the video if a live video stream is being sent.

A drone can fly without a telemetry module with only the R/C uplink but doing so is risky because the pilot would have no idea about the status of the components on the drone, most importantly, the battery level.

---

#### 5 What types of batteries are used in a drone and why? Give a brief explanation.

Drone batteries must have a high storage capacity for higher flight time and a large power output. Apart from this, they need to be small, lightweight, and not get too hot when used. There are four types of drone batteries:

- Lithium-Polymer

  The Lipo battery is the most common in almost all electric drones. It's compact, lightweight, and has a large energy capacity to operate any drone. Moreover, when in storage, it has a slower discharge rate which means it can hold charge better. This is useful because one can stay long without using them without worrying about them discharging.

- Nickel Cadmium

  These batteries existed before the lipo batteries and can't give very high currents like lipo batteries. They also have a small energy capacity which makes them unable to hold a charge for long, hence shorter life expectancy, and lower energy density.
  They also suffer from the "memory effect." This occurs when the battery is repeatedly charged without being fully discharged, causing it to "remember" its previous charge capacity and gradually lose its ability to hold a charge. This can result in reduced battery life and performance over time. NiCd batteries are also less environmentally friendly than other types of batteries, as they contain toxic materials that can harm the environment if not disposed of properly.

- Lithium High Voltage (LiHv)

  Compared to the 4.2V output of Lipo batteries, LiHv batteries can provide 4.35V. Hence they are used in racing and nano drones because they function well with high voltage.

- Brand-Specific

  Manufacturers like DJI design their own batteries for their drones which come with advanced features like telemetry, safety features etc. Their drones work with only their batteries.

Specs related to lipo battery:

- S rating

  This refers to the number of cells. Each cell can have a maximum voltage of 4.2 Volts and a (suggested) minimum voltage of 3.7 Volts. For example, a 3S battery will be three cells connected in series, giving a max voltage of 12.6 V and a min voltage of 11.1 V. Racing drones generally use 4S upto 6S batteries as they need more power. In contrast, other drones looking for more flight time go for 3S batteries.

- mAh

  Explaining with an example, a 5000mah battery which is 5 Amp-hour, can supply 5 Amps of current for an hour, after which it's completely drained. So if this battery is put on a quadcopter with the motors continuously consuming 50 Amps of power, the drone can run for around 6 minutes.

- C rating

  It's the cell's maximum discharge rate. Say the battery mentioned above has a C rating of 20. This means the maximum current we can pull out of it is 5A \* 20 = 100 Amps.

The disadvantages of the lipo battery are:

- Doesn't function properly in cold weather
- Delicate construction

  Any kind of puncture can lead to an explosion. This can happen if the battery is tightly mounted on the drone and a screw or some other pointed object pierces it.

- Battery swelling

  When a lipo battery is constantly used for long periods, especially when it's overcharged or over discharged, electrolyte decomposition occurs, leading to oxygen gas buildup, which is a fire risk. Since the outer covering of the battery is completely sealed, the gas can't escape, and the battery puffs up.
