package frc.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

public class FireControlUtil {

    private final Pose2d speakerPose;
    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();
    private final PIDController pidController;

    public FireControlUtil(boolean allianceIsRed){
        if (allianceIsRed) speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        else speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(7).get().toPose2d(); // Default to blue alliance

        //TODO: Put the new values from the arm here
        interpolator.put(0.0, 0.0);

        //TODO: Get actual values for the PID controller
        pidController = new PIDController(0.1, 0.0, 0.0);
    }
    public double getShooterAngle(Pose2d robotPose) {
        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        return interpolator.get(distance);
    }

    public double getPIDValue(Pose2d robotPose, Rotation2d currentHeading){
        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        Rotation2d rotToSpeaker = transformToSpeaker.getRotation();

        return pidController.calculate(rotToSpeaker.getDegrees(), currentHeading.getDegrees());
    }
}
