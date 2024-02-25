package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonClass extends SubsystemBase {
    // Camera whose name is declared in constants
    private final PhotonCamera camera;

    private final Transform3d robotToCamera;

    public PhotonClass(String cameraName, Transform3d robotToCam) {
        // Sets up the camera, and determines which AprilTags will be used for estimation
        camera = new PhotonCamera(cameraName);
        robotToCamera = robotToCam;
    }

    // Gets a result from PhotonVision via NetworkTables
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public PhotonTrackedTarget getAprilTag(int tagID) {
        var targets = getLatestResult().getTargets();
        PhotonTrackedTarget desiredTarget = null;
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == tagID) {
                desiredTarget = target;
            }
        }
        return desiredTarget;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Transform3d getRobotToCam() {
        return robotToCamera;
    }
}