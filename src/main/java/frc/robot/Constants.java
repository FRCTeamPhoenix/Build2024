// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final class General {
        //Logging Constant
        public static final boolean LOGGING = false;

    }

    public static final class VisionConstants {
//        public static final String kGenericCameraName = "Arducam_OV9281_USB_Camera";

        public static final String kFrontCameraName = "front_arducam";
        public static final String kRearCameraName = "back_arducam";
        public static final String kLeftCameraName = "left_arducam";
        public static final String kRightCameraName = "right_arducam";


        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kFrontTransform = new Transform3d(new Translation3d(0.3048, 0.0, 0.12065), new Rotation3d(0, Math.toRadians(20), 0.0));
        public static final Transform3d kRearTransform = new Transform3d(new Translation3d(-0.3048, 0.0, 0.12065), new Rotation3d(0, Math.toRadians(20), Math.PI));
        public static final Transform3d kLeftTransform = new Transform3d(new Translation3d(0.0, -0.3048, 0.12065), new Rotation3d(0, Math.toRadians(20), (Math.PI / 2.0)));
        public static final Transform3d kRightTransform = new Transform3d(new Translation3d(0.0, 0.3048, 0.12065), new Rotation3d(0, Math.toRadians(20), (-Math.PI / 2.0)));

        // TODO: Add left when necessary

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class DriveConstants {


        // VERY IMPORTANT!!!!!!
        // Neos = 1, Falcons = 2, Krakens = 3
        public static final int motorType = 3;
        //Are we using Pigeon2 or Pigeon
        public static final boolean usingPigeon2 = true;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 6.72;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0.0;
        public static final double kBackLeftChassisAngularOffset = -Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // MOTOR CAN IDs
        public static final int kFrontLeftTurningCanId = 1;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 3;
        public static final int kRearLeftTurningCanId = 4;

        public static final int kFrontLeftDrivingCanId = 6;
        public static final int kFrontRightDrivingCanId = 5;
        public static final int kRearRightDrivingCanId = 7;
        public static final int kRearLeftDrivingCanId = 8;

        public static final int kPigeonCanId = 9;

        public static final boolean kGyroReversed = false;
    }

    public static final class ClimberConstants {
        public static final IdleMode kClimberIdleMode = IdleMode.kBrake;
        public static final int kClimberLeftCanId = 15;
        public static final int kClimberRightCanId = 16;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kNeoDrivingMotorPinionTeeth = 12;
        public static final int kFalconDrivingMotorPinionTeeth = 16;
        public static final int kKrakenDrivingMotorPinionTeeth = 16;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kNeoDrivingMotorReduction = (45.0 * 22) / (kNeoDrivingMotorPinionTeeth * 15);
        public static final double kNeoDriveWheelFreeSpeedRps = (kNeoDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kNeoDrivingMotorReduction;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kFalconDrivingMotorFreeSpeedRps = 6380;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kFalconDrivingMotorReduction = (45.0 * 19) / (kFalconDrivingMotorPinionTeeth * 15);
        public static final double kFalconDriveWheelFreeSpeedRps = (kFalconDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kFalconDrivingMotorReduction;

        public static final double kKrakenDrivingMotorFreeSpeedRps = 6000;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kKrakenDrivingMotorReduction = (45.0 * 19) / (kKrakenDrivingMotorPinionTeeth * 15);
        public static final double kKrakenDriveWheelFreeSpeedRps = (kKrakenDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kKrakenDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kNeoDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kNeoDrivingMotorReduction) / 60.0; // meters per second

        public static final double kFalconEncoderFactor = ((kWheelDiameterMeters * Math.PI)
                / kFalconDrivingMotorReduction);
        
        public static final double kKrakenEncoderFactor = ((kWheelDiameterMeters * Math.PI)
                / kKrakenDrivingMotorReduction);

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kNeoDrivingP = 0.04;
        public static final double kNeoDrivingI = 0;
        public static final double kNeoDrivingD = 0;
        public static final double kNeoDrivingFF = 1 / kNeoDriveWheelFreeSpeedRps;
        public static final double kNeoDrivingMinOutput = -1;
        public static final double kNeoDrivingMaxOutput = 1;

        public static final double kTalonDrivingP = 0.11; // An error of 1 rotation per second results in 2V output
        public static final double kTalonDrivingI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        public static final double kTalonDrivingD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        public static final double kTalonDrivingV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        public static final double kTalonDrivingPeakForwardVoltage = 12;
        public static final double kTalonDrivingPeakReverseVoltage = -12;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final NeutralModeValue kDrivingMotorNeutralMode = NeutralModeValue.Brake;
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 30; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.1;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class IntakeConstants {
        public static final double kIntakeWheelDiameterMeters = 0.0508;
        public static final double kIntakeWheelCircumferenceMeters = 0.0508 * Math.PI;
        public static final double kIntakeMotorReduction = 3.0;

        public static final double kIntakeFreeSpeedRps = (NeoMotorConstants.kFreeSpeedRps * kIntakeWheelCircumferenceMeters)
                / kIntakeMotorReduction;

        public static final double kIntakeEncoderPositionFactor = kIntakeWheelCircumferenceMeters
                / kIntakeMotorReduction; // meters
        public static final double kIntakeEncoderVelocityFactor = (kIntakeWheelCircumferenceMeters
                / kIntakeMotorReduction) / 60.0; // meters per second

        public static final double kIntakeP = 0.04;
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
        public static final double kIntakeFF = 1 / kIntakeFreeSpeedRps;
        public static final double kIntakeMinOutput = -1;
        public static final double kIntakeMaxOutput = 1;

        public static final int kIntakeMotorCurrentLimit = 40; // amps

        public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    }

    public static final class ShooterConstants {
        public static final double kShooterWheelDiameterMeters = 0.1016;
        public static final double kShooterWheelCircumferenceMeters = 0.1016 * Math.PI;
        public static final double kShooterMotorReduction = 1.0;

        public static final double kShooterFreeSpeedRps = (NeoMotorConstants.kFreeSpeedRps * kShooterWheelCircumferenceMeters)
                / kShooterMotorReduction;

        public static final double kShooterEncoderPositionFactor = kShooterWheelCircumferenceMeters
                / kShooterMotorReduction; // meters
        public static final double kShooterEncoderVelocityFactor = (kShooterWheelCircumferenceMeters
                / kShooterMotorReduction) / 60.0; // meters per second

        public static double kShooterP = 0.04;
        public static double kShooterI = 0.0;
        public static double kShooterD = 0.0;
        public static final double kShooterFF = 1 / kShooterFreeSpeedRps;
        public static final double kShooterMinOutput = -1;
        public static final double kShooterMaxOutput = 1;

        public static final int kShooterMotorCurrentLimit = 40; // amps

        public static final IdleMode kShooterMotorIdleMode = IdleMode.kBrake;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
        public static final double kFreeSpeedRps = 5676 / 60;
    }

    public static final class ArmConstants {
        //public static final double ARM_MIN_ANGLE = 0.08;
        public static final double ARM_MIN_ANGLE = 0.068;
        public static final double ARM_MAX_ANGLE = 2.95;
        public static final double ARM_CLIMBER_ANGLE = 1.99;
        public static final double ARM_SUBWOOFER_ANGLE = 0.4;
        public static final double ARM_BALANCED_ANGLE = 0.96;
        public static final double kArmP = 2.25;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.1;
        public static final double kArmFF = 0.0;
        public static final double ArmMoveSetPoint = .25;
        public static final double kArmMinOutput = -1;
        public static final double kArmMaxOutput = 1;
        public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
        public static final int kArmMotorCurrentLimit = 30; // amps

        public static final double kArmEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
        public static final double kArmEncoderPositionPIDMinInput = 0; // radians
        public static final double kArmEncoderPositionPIDMaxInput = kArmEncoderPositionFactor; // radians
    }
}
