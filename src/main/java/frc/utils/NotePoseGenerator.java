package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NotePoseGenerator {

    public static Pose2d generateNotePose(OakCameraObject trackedNote){
        double x = trackedNote.getHorizontalDistance() * Math.cos(Math.toRadians(trackedNote.getXAngle()));
        double y = trackedNote.getHorizontalDistance() * Math.sin(Math.toRadians(trackedNote.getXAngle()));
        Pose2d notePose = new Pose2d(x, y, new Rotation2d(Math.toRadians(trackedNote.getXAngle())));
        return notePose;
    }
}
