package frc.utils;
import java.lang.Math;

public class CameraDriveUtil {

    public static double getDriveX(double theta, double distance, double desiredXDistance){
        // These numbers must be tuned for your Robot!  Be careful!
        final double xDriveSpeed = 0.4;                    // How hard to drive fwd toward the target
        final double MaxXDriveVelocity = 0.14;    // maximum allowed drive velocity

        theta = 360 - theta;

        if (theta > 180){
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
        }
        else if (xVelocity < -MaxXDriveVelocity) {
          xVelocity = -MaxXDriveVelocity;
        }
        return xVelocity;
    }
    public static double getDriveY(double theta, double distance, double desiredYDistance){ 
        // These numbers must be tuned for your Robot!  Be careful!
        final double yDriveSpeed = 0.4;                    // How hard to drive fwd toward the target
        final double MaxYDriveVelocity = 0.14;    // maximum allowed drive velocity

        theta = 360 - theta;

        if (theta > 180){
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
        }
        else if (xVelocity < -MaxYDriveVelocity) {
          xVelocity = -MaxYDriveVelocity;
        }
        return xVelocity;
    }

    public static double getDriveRot(double theta, double DesiredTheta){          
        // These numbers must be tuned for your Robot!  Be careful!
        final double turnSpeed = 0.01;                  // How hard to turn toward the target
        final double maxTurnVelocity = 0.03;    // Maximum allowed rotational velocity
 
        // if angle within 2 degrees of desired angle set error to 0
        theta = 360 - theta;
        
        if (theta > 180){
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
        }
        else if (thetaVelocity < -maxTurnVelocity) {
            thetaVelocity = -maxTurnVelocity;
        }
        return thetaVelocity;
    }
        
}