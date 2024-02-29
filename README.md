# Build2024: 2342's Code for <i>Crescendo</i>
A swerve drive using MAXSwerve modules with a Quokkas-style intake and shooter on a pivoting arm.


<h2>Software Information</h2>
WPILib Version: 2024.3.1

Latest Phoenix 6 Firmware

<h3>Libraries Used</h3>
<ul>
    <li>CTRE-Phoenix (v5)</li>
    <li>CTRE-Phoenix (v6)</li>
    <li>PathplannerLib</li>
    <li>photonlib</li>
    <li>REVLib</li>
    <li>WPILib-New-Commands</li>
</ul>


<h2>Hardware Requirements</h2>
<h3>Supported Driving Motors</h3>
<ul>
    <li>NEO v1.1 with Spark MAX
    <li>Falcon 500
    <li>Kraken X60
</ul>
The IMU is a Pigeon 2 with Phoenix 6 or Pigeon [1] with Phoenix 5.

The Turning Motors are NEO 550s with a Rev Through Bore Encoder plugged into the Spark MAX.

The Arm is driven with 2 NEO v1.1s with a Rev Through Bore Encoder plugged into the left motor's Spark MAX.

The Shooter Motors are 2 NEO v1.1s.

The Intake uses a single NEO v1.1.

The Intake also has a SparkFun Proximity Sensor to detect if a note has been captured.

<h2>Vision</h2>

The Vision is split into two primary subsystems:
<h3>PhotonVision</h3>
PhotonVision is running off an Orange Pi with multiple ArduCam OV9281 USB Camera.

PhotonVision is primarily used to detect AprilTags and calculate our robot pose and align the shooter with the speaker.

<h3>OAK Cameras</h3>
The OAK-D Pro W cameras are connected to a Raspberry Pi on our robot.

The OAK Cameras are used in the detection of notes to intake notes during autonomous.
