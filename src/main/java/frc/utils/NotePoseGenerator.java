package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NotePoseGenerator {

    public static Pose2d generateNotePose(OakCameraObject trackedNote, Pose2d currentPose){
        double x = trackedNote.getHorizontalDistance() * Math.cos(Math.toRadians(trackedNote.getXAngle()));
        x = (x/1000) + currentPose.getX();
        double y = trackedNote.getHorizontalDistance() * Math.sin(Math.toRadians(trackedNote.getXAngle()));
        y = (y/1000) + currentPose.getY();
        Pose2d notePose = new Pose2d(x, y, new Rotation2d(Math.toRadians(trackedNote.getXAngle())).plus(currentPose.getRotation()));
        return notePose;
    }
}
