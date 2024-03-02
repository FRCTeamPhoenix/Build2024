package frc.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FireControlUtil {

    private final Pose2d speakerPose;
    private final InterpolatingDoubleTreeMap interpolator = new InterpolatingDoubleTreeMap();
    private final PIDController pidController;
    private final Field2d field = new Field2d();

    public FireControlUtil(boolean allianceIsRed){
        if (allianceIsRed) speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(4).get().toPose2d();
        else speakerPose = Constants.VisionConstants.kTagLayout.getTagPose(7).get().toPose2d(); // Default to blue alliance

        //TODO: Put the new values from the arm here
        interpolator.put(0.0, 0.0);

        //TODO: Get actual values for the PID controller
        pidController = new PIDController(0.00015, 0.0, 0.0);
    }
    public double getShooterAngle(Pose2d robotPose) {
        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        return interpolator.get(distance);
    }

    public double getPIDValue(Pose2d robotPose, Rotation2d currentHeading) {
        double deltaX = robotPose.getX() - speakerPose.getX();
        double deltaY = robotPose.getY() - speakerPose.getY();
        double angle = Math.atan2(deltaY, deltaX);

        field.setRobotPose(speakerPose);

        SmartDashboard.putData("SpeakerPose", field);

        SmartDashboard.putNumber("Speaker Rot", Math.toDegrees(angle));
        return pidController.calculate(currentHeading.getDegrees(), Math.toDegrees(angle) + 180.0);
    }
}
