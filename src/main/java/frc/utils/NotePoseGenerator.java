package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NotePoseGenerator {
    public static Pose2d generateNotePose(OakCameraObject trackedNote, Pose2d currentPose) {
        double distance = trackedNote.getHorizontalDistance() / 1000.0;
        double noteAngle = trackedNote.getXAngle();

        double angle = currentPose.getRotation().getDegrees() - noteAngle;

        if (angle < 0) angle += 360;

        double x = distance * Math.cos(Math.toRadians(angle)) + currentPose.getX();
        double y = distance * Math.sin(Math.toRadians(angle)) + currentPose.getY();

        return new Pose2d(x, y, new Rotation2d(0.0));
    }
}
