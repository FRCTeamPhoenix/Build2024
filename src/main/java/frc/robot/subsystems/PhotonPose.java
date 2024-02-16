package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import static frc.robot.Constants.VisionConstants;
public class PhotonPose extends SubsystemBase {
    // Camera whose name is declared in constants
    private final PhotonClass camera;
 
    // The timestamp used to mark each timestamp
    private double lastEstTimestamp = 0;

    // Supplies poses to the SwerveDrivePoseEstimator
    private final PhotonPoseEstimator photonEstimator;

    // Does the actual pose estimation using vision and odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    private final DriveSubsystem drive;

    Field2d field2d = new Field2d();

    public PhotonPose(DriveSubsystem drive, PhotonClass cameraClass) {
        // Sets up the camera, and determines which AprilTags will be used for estimation
        camera = cameraClass;
        photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera.getCamera(), camera.getRobotToCam());
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        this.drive = drive;
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            drive.getRotation(), 
            drive.getModulePositions(), 
            new Pose2d());
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp-lastEstTimestamp) > 1e-5;
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

    public Pose2d getPose(){
        var visionEst = getEstimatedGlobalPose();
    
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs(estPose);

                    poseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            
                });
        poseEstimator.update(drive.getRotation(), drive.getModulePositions());
        return poseEstimator.getEstimatedPosition();
    }

    @Override

    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
                // Correct pose estimate with vision measurements
        var visionEst = getEstimatedGlobalPose();
    
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs(estPose);

                    poseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            
                });
        poseEstimator.update(drive.getRotation(), drive.getModulePositions());
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", field2d);
        double[] pose = {poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), poseEstimator.getEstimatedPosition().getRotation().getRadians()};
        SmartDashboard.putNumberArray("Me pose", pose);
    }
    
}