package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FireControlUtil {

    private final Pose2d speakerPose;
    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();
    private final PIDController pidController;
    private final PIDController otherPID;
    private final Field2d field = new Field2d();

    public FireControlUtil(boolean allianceIsRed) {
        if (allianceIsRed) speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        else
            speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(7).get().toPose2d(); // Default to blue alliance

        //TODO: Put the new values from the arm here
        // Modifying calculated values to fine-tune angles (03/03/2024)
        interpolator.put(0.61, 0.364);
        interpolator.put(1.63, 0.514);
        interpolator.put(2.35, 0.674);
        interpolator.put(2.85, 0.724);
        interpolator.put(3.40, 0.794);
        interpolator.put(3.92, 0.804);
        interpolator.put(4.58, 0.834);


        // Calculated values for updated arm (03/02/2024)
//        interpolator.put(0.60652, 0.3497);
//        interpolator.put(1.6333, 0.4969);
//        interpolator.put(2.3544, 0.6624);
//        interpolator.put(2.8532, 0.7114);
//        interpolator.put(3.3961, 0.7788);
//        interpolator.put(3.9187, 0.7791);
//        interpolator.put(4.5754, 0.8218);


        //TODO: Get actual values for the PID controller
        pidController = new PIDController(0.015, 0.0, 0.0);
        otherPID = new PIDController(0.015, 0.0, 0.0);
        otherPID.enableContinuousInput(0, 360);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getShooterAngle(Pose2d robotPose, double currentArmPosition) {
        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        if (distance <= 4.5754 && distance >= 0.60652) {
            return interpolator.get(distance);
        } else return currentArmPosition;
    }

    public double getPIDValue(Pose2d robotPose, Rotation2d currentHeading) {
        double deltaX = robotPose.getX() - speakerPose.getX();
        double deltaY = robotPose.getY() - speakerPose.getY();
        double angle = Math.atan2(deltaY, deltaX);

        field.setRobotPose(speakerPose);

        double number = pidController.calculate(MathUtil.angleModulus(currentHeading.getRadians()), angle);

        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("pdi", number);

        return number;
    }

    public double getPIDWithFeedForward(Pose2d robotPose, Rotation2d currentHeading, DriveSubsystem drive) {
        double deltaX = robotPose.getX() - speakerPose.getX();
        double deltaY = robotPose.getY() - speakerPose.getY();
        double angle = Math.atan2(deltaY, deltaX);

        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        double ff = (drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond / distance);

        field.setRobotPose(speakerPose);

        return otherPID.calculate(MathUtil.angleModulus(currentHeading.getRadians()), angle) + ff;
    }

    public double turnToDirection(Rotation2d currentHeading, double desiredAngle) {
        return pidController.calculate(currentHeading.getDegrees(), desiredAngle);
    }
}
