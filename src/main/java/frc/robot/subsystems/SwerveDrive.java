package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveDrive {
    private MAXSwerveModule sparkMax;

    private TalonSwerveModule talon;

    public SwerveDrive(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        if (Constants.DriveConstants.motorType == 2 || DriveConstants.motorType == 3) {
            talon = new TalonSwerveModule(drivingCANId, turningCANId, chassisAngularOffset);
        } else {
            sparkMax = new MAXSwerveModule(drivingCANId, turningCANId, chassisAngularOffset);
        }
    }

    public SwerveModulePosition getPosition() {
        if (Constants.DriveConstants.motorType == 2 || DriveConstants.motorType == 3) {
            return talon.getPosition();
        } else {
            return sparkMax.getPosition();
        }
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Constants.DriveConstants.motorType == 2 || DriveConstants.motorType == 3) {
            talon.setDesiredState(state);
        } else {
            sparkMax.setDesiredState(state);
        }
    }

    public void resetEncoders() {
        if (Constants.DriveConstants.motorType == 2 || DriveConstants.motorType == 3) {
            talon.resetEncoders();
        } else {
            sparkMax.resetEncoders();
        }
    }

    public SwerveModuleState getState() {
        if (Constants.DriveConstants.motorType == 2 || DriveConstants.motorType == 3) {
            return talon.getState();
        } else {
            return sparkMax.getState();

        }
    }

    public TalonFX getTalon() {
        return talon.getTalon();
    }
}