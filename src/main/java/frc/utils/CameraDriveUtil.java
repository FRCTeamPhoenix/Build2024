package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.lang.Math;

public class CameraDriveUtil {

    public static double getDriveX(double theta, double distance, double desiredXDistance) {
        // These numbers must be tuned for your Robot!  Be careful!
        final double xDriveSpeed = 0.4;                    // How hard to drive fwd toward the target
        final double MaxXDriveVelocity = 0.14;    // maximum allowed drive velocity

        theta = 360 - theta;

        if (theta > 180) {
            theta -= 360;
        }

        double xDistance = distance * Math.cos(Math.toRadians(theta));

        // if distance within 0.15 of desired distance set error to 0
        double xError = xDistance - desiredXDistance;
        if (Math.abs(xError) < 200) {
            xError = 0;
        }

        // calculates theoretical X velocity 
        double xVelocity = (xError * xDriveSpeed);
        // caps the X velocity to the maximum allowed velocity
        if (xVelocity > MaxXDriveVelocity) {
            xVelocity = MaxXDriveVelocity;
        } else if (xVelocity < -MaxXDriveVelocity) {
            xVelocity = -MaxXDriveVelocity;
        }
        return xVelocity;
    }

    public static double getDriveY(double theta, double distance, double desiredYDistance) {
        // These numbers must be tuned for your Robot!  Be careful!
        final double yDriveSpeed = 0.4;                    // How hard to drive fwd toward the target
        final double MaxYDriveVelocity = 0.14;    // maximum allowed drive velocity

        theta = 360 - theta;

        if (theta > 180) {
            theta -= 360;
        }

        double yDistance = distance * Math.sin(Math.toRadians(theta));

        // if distance within 0.15 of desired distance set error to 0
        double yError = yDistance - desiredYDistance;
        if (Math.abs(yError) < 200) {
            yError = 0;
        }

        // calculates theoretical X velocity 
        double xVelocity = (yError * yDriveSpeed);
        // caps the X velocity to the maximum allowed velocity
        if (xVelocity > MaxYDriveVelocity) {
            xVelocity = MaxYDriveVelocity;
        } else if (xVelocity < -MaxYDriveVelocity) {
            xVelocity = -MaxYDriveVelocity;
        }
        return xVelocity;
    }

    public static double getDriveRot(double theta, double DesiredTheta) {
        // These numbers must be tuned for your Robot!  Be careful!
        final double turnSpeed = 0.05;                  // How hard to turn toward the target
        final double maxTurnVelocity = 0.15;    // Maximum allowed rotational velocity

        // if angle within 2 degrees of desired angle set error to 0
        theta = 360 - theta;

        if (theta > 180) {
            theta -= 360;
        }

        double thetaError = DesiredTheta - theta;
        if (Math.abs(thetaError) < 5) {
            thetaError = 0;
        }

        // calculates theoretical rotational velocity
        double thetaVelocity = (thetaError * turnSpeed);
        // caps the rotaional velocity to the maximum allowed velocity
        if (thetaVelocity > maxTurnVelocity) {
            thetaVelocity = maxTurnVelocity;
        } else if (thetaVelocity < -maxTurnVelocity) {
            thetaVelocity = -maxTurnVelocity;
        }
        return thetaVelocity;
    }

    public static double getDriveRotWithFeedForward(double theta, double DesiredTheta, Pose2d previousPose, Pose2d currentPose) {
        // These numbers must be tuned for your Robot!  Be careful!
        final double turnSpeed = 0.05;                  // How hard to turn toward the target
        final double maxTurnVelocity = 0.15;    // Maximum allowed rotational velocity

        //TODO: Get actual speaker pose
        Pose2d speakerPose = new Pose2d(0, 0, new Rotation2d(10));

        // if angle within 2 degrees of desired angle set error to 0
        theta = 360 - theta;

        if (theta > 180) {
            theta -= 360;
        }

        double thetaError = DesiredTheta - theta;
        if (Math.abs(thetaError) < 1) {
            thetaError = 0;
        }

        // calculates theoretical rotational velocity
        double thetaVelocity = (thetaError * turnSpeed);
        // caps the rotaional velocity to the maximum allowed velocity
        if (thetaVelocity > maxTurnVelocity) {
            thetaVelocity = maxTurnVelocity;
        } else if (thetaVelocity < -maxTurnVelocity) {
            thetaVelocity = -maxTurnVelocity;
        }

        //Calculating feed forward
        Transform2d currentTransform = new Transform2d(currentPose, speakerPose);
        Transform2d priorTransform = new Transform2d(previousPose, speakerPose);
        Transform2d deltaTransform = new Transform2d(
                currentTransform.getX() - priorTransform.getX(),
                currentTransform.getY() - priorTransform.getY(),
                new Rotation2d(currentTransform.getRotation().getRadians() - priorTransform.getRotation().getRadians())
        );

        double feedForward = deltaTransform.getRotation().getRadians(); //radians/20ms cycle
        feedForward = feedForward * 50; // radians/sec

        //Adding feedforward to velocity
        thetaVelocity += feedForward;
        return thetaVelocity;
    }

}