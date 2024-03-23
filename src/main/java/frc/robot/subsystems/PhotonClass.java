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

public class PhotonClass extends SubsystemBase {
    // Camera whose name is declared in constants
    private final PhotonCamera camera;

    private final Transform3d robotToCamera;

    // The timestamp used to mark each timestamp
    private double lastEstTimestamp = 0;

    // Supplies poses to the SwerveDrivePoseEstimator
    private final PhotonPoseEstimator photonEstimator;

    public PhotonClass(String cameraName, Transform3d robotToCam) {
        // Sets up the camera, and determines which AprilTags will be used for estimation
        camera = new PhotonCamera(cameraName);
        robotToCamera = robotToCam;
        photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Transform3d getRobotToCam() {
        return robotToCamera;
    }
}