package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// This is adapted from PhotonClass.java
public class PhotonNote extends SubsystemBase {
    // Camera whose name is declared in constants
    private final PhotonCamera camera;

    // The timestamp used to mark each timestamp
    private double lastEstTimestamp = 0;

    public PhotonNote(String cameraName) {
        // Sets up the camera, and determines which AprilTags will be used for estimation
        camera = new PhotonCamera(cameraName);
    }

    // Gets a result from PhotonVision via NetworkTables
    public PhotonPipelineResult getNoteResult() {
        return camera.getLatestResult();
    }

    public PhotonTrackedTarget getNearestNote() {
        var targets = getNoteResult();
        PhotonTrackedTarget NearestNote = null;
        if (targets.hasTargets()) {    //PhotonVision docs say ALWAYS call hasTargets
            NearestNote = targets.getBestTarget();
        }
        return NearestNote;
    }
    // This function returns either actual +/- yaw value in degrees
    // if note is detected or 180 if no note detected.  The windows GUI
    // is CW+, PhotonVision doc says CCW+
    public double getNearestNoteYaw() {
        var note = getNearestNote();
        if (note != null)
            return(note.getYaw());
        else 
            return (180);
    }

    public PhotonCamera getCamera() {
        return camera;
    }

}