package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveDrive {
  private MAXSwerveModule sparkMax;

  private TalonSwerveModule talon;

  public SwerveDrive(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    if (Constants.DriveConstants.usingTalons) {
        talon = new TalonSwerveModule(drivingCANId, turningCANId, chassisAngularOffset);
    }
    else {
        sparkMax = new MAXSwerveModule(drivingCANId, turningCANId, chassisAngularOffset);
    }
  }

   public SwerveModulePosition getPosition() {
    if (Constants.DriveConstants.usingTalons) {
        return talon.getPosition();
    }
    else {
        return sparkMax.getPosition();
    }
   }

   public void setDesiredState(SwerveModuleState state) {
    if (Constants.DriveConstants.usingTalons) {
        talon.setDesiredState(state);
    }
    else {
        sparkMax.setDesiredState(state);
    }
   }

   public void resetEncoders() {
    if (Constants.DriveConstants.usingTalons) {
        talon.resetEncoders();
    }
    else {
        sparkMax.resetEncoders();
    }
   }

   public SwerveModuleState getState() {
    if (Constants.DriveConstants.usingTalons) {
        return talon.getState();
    }
    else {
        return sparkMax.getState();

    }
   }
}