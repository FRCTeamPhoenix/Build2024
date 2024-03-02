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
        interpolator.put(0.60652, 0.3497);
        interpolator.put(1.6333, 0.4969);
        interpolator.put(2.3544, 0.6624);
        interpolator.put(2.8532, 0.7114);
        interpolator.put(3.3961, 0.7788);
        interpolator.put(3.9187, 0.7791);
        interpolator.put(4.5754, 0.8218);


        //TODO: Get actual values for the PID controller
        pidController = new PIDController(0.00015, 0.0, 0.0);
    }
    public double getShooterAngle(Pose2d robotPose, double currentArmPosition) {
        Transform2d transformToSpeaker = robotPose.minus(speakerPose);
        double distance = Math.sqrt(Math.pow(transformToSpeaker.getX(), 2) + Math.pow(transformToSpeaker.getY(), 2));
        if (distance <= 4.5754 && distance >= 0.60652) {
            SmartDashboard.putNumber("InterpolatorDistance", interpolator.get(distance));
            return interpolator.get(distance);
        }
        else return currentArmPosition;
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
