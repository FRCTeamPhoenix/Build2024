package frc.robot.subsystems;


//import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

private String deviceID;
private boolean m_LimelightHasValidTarget = false;
private double m_LimelightDriveX = 0.0;
private double m_LimelightDriveY = 0.0;
private double m_LimelightDriveRot = 0.0;
private double m_targetArea = 0.0;
private double m_distanceToTarget;

    public LimeLight(String deviceID) {
        this.deviceID = deviceID;
    }

    public void limeLight() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Update_Limelight_Tracking();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void Update_Limelight_Tracking(){
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.01;                  // How hard to turn toward the target
        final double DRIVE_K = 0.4;                    // How hard to drive fwd toward the target
        final double DESIRED_TARGET_DISTANCE = 0.5;
        final double DESIRED_HEADING = 0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.4;
        final double MAX_STEER = 0.1;

        double tv = NetworkTableInstance.getDefault().getTable(deviceID).getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable(deviceID).getEntry("tx").getDouble(0);
        //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable(deviceID).getEntry("ta").getDouble(0);
        m_targetArea = ta;
        double distance = 1 / Math.sqrt(ta);
        if (tv != 0) m_LimelightHasValidTarget = true;
        else m_LimelightHasValidTarget = false;

        double errDistance = distance - DESIRED_TARGET_DISTANCE;
        if (Math.abs(errDistance) < 0.15) {
          errDistance = 0;
        }
        m_distanceToTarget = distance;

        double errAngle = DESIRED_HEADING - tx;
        if (Math.abs(errAngle) < 2) {
          errAngle = 0;
        }

        double speed = Math.abs(DRIVE_K * errDistance);
        double x = (errDistance * Math.cos(errAngle * (Math.PI / 180))) * speed;
        if (x > MAX_DRIVE) {
          x = MAX_DRIVE;
        }
        else if (x < -MAX_DRIVE) {
          x = -MAX_DRIVE;
        }
        m_LimelightDriveX = x;

        double y = (errDistance * Math.sin(errAngle * (Math.PI / 180))) * speed;
        if (y > MAX_DRIVE) {
          y = MAX_DRIVE;
        }
        else if (y < -MAX_DRIVE) {
          y = -MAX_DRIVE;
        }
        m_LimelightDriveY = y;

        double rotate = (errAngle * STEER_K);
        if (rotate > MAX_STEER) rotate = MAX_STEER;
        else if (rotate < -MAX_STEER) rotate = -MAX_STEER;
        m_LimelightDriveRot = rotate;
  }

  public boolean hasValidTarget() {
      return m_LimelightHasValidTarget;
  }

  public double getLLDriveX() {
    return m_LimelightDriveX;
  }

  public double getLLDriveY() {
    return m_LimelightDriveY;
  }


  public double getLLDriveRotation() {
    return m_LimelightDriveRot;
  }

  public double getLLTargetArea() {
    return m_targetArea;
  }

  public double getLLTargetDistance() {
    return m_distanceToTarget;
  }
}

