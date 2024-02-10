package frc.utils;

public class CameraDriveUtil {

    private static double m_DriveX = 0.0;
    private static double m_DriveY = 0.0;
    private static double m_DriveRot = 0.0;
    private static double m_HorizontalDistance;


    public static void CalculateDriveValues(double thetaX, double thetaY, double distance, double DESIRED_TARGET_DISTANCE, double DESIRED_HEADING){ 
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_V = 0.01;                  // How hard to turn toward the target
        final double DRIVE_V = 0.4;                    // How hard to drive fwd toward the target
        final double MAX_DRIVE = 0.4;    // maximum allowed drive velocity
        final double MAX_TURN = 0.1;    // Maximum allowed rotational velocity
        
        // calculates horizontal distance to target 
        m_HorizontalDistance = distance * Math.cos(thetaY * (Math.PI / 180));

        // if distance within 0.15 of desired distance set error to 0
        double errDistance = m_HorizontalDistance - DESIRED_TARGET_DISTANCE;
        if (Math.abs(errDistance) < 100) {
          errDistance = 0;
        }

        // if angle within 2 degrees of desired angle set error to 0
        double errAngle = DESIRED_HEADING - thetaX;
        if (Math.abs(errAngle) < 2) {
          errAngle = 0;
        }
        // calculates robot movment velocity baised on current distance from target
        double speed = Math.abs(DRIVE_V * errDistance);

        // calculates theoretical X velocity 
        double x = (errDistance * Math.cos(errAngle * (Math.PI / 180))) * speed;
        // caps the X velocity to the maximum allowed velocity
        if (x > MAX_DRIVE) {
          x = MAX_DRIVE;
        }
        else if (x < -MAX_DRIVE) {
          x = -MAX_DRIVE;
        }
        m_DriveX = x;

        // calculates theoretical Y velocity 
        double y = (errDistance * Math.sin(errAngle * (Math.PI / 180))) * speed;
        // caps the Y velocity to the maximum allowed velocity
        if (y > MAX_DRIVE) {
          y = MAX_DRIVE;
        }
        else if (y < -MAX_DRIVE) {
          y = -MAX_DRIVE;
        }
        m_DriveY = y;
        
        // calculates theoretical rotational velocity
        double rotate = (errAngle * STEER_V);
        // caps the rotaional velocity to the maximum allowed velocity
        if (rotate > MAX_TURN) {
            rotate = MAX_TURN;
        }
        else if (rotate < -MAX_TURN) {
            rotate = -MAX_TURN;
        }
        m_DriveRot = rotate;
    }

    public static double getDriveX(double thetaX, double thetaY, double distance, double DESIRED_TARGET_DISTANCE, double DESIRED_HEADING){
        // These numbers must be tuned for your Robot!  Be careful!
        final double DRIVE_V = 0.4;                    // How hard to drive fwd toward the target
        final double MAX_DRIVE = 0.4;    // maximum allowed drive velocity
        
        // calculates horizontal distance to target 
        m_HorizontalDistance = distance * Math.cos(thetaY * (Math.PI / 180));

        // if distance within 0.15 of desired distance set error to 0
        double errDistance = m_HorizontalDistance - DESIRED_TARGET_DISTANCE;
        if (Math.abs(errDistance) < 100) {
          errDistance = 0;
        }

        // if angle within 2 degrees of desired angle set error to 0
        double errAngle = DESIRED_HEADING - thetaX;
        if (Math.abs(errAngle) < 2) {
          errAngle = 0;
        }
        // calculates robot movment velocity baised on current distance from target
        double speed = Math.abs(DRIVE_V * errDistance);

        // calculates theoretical X velocity 
        double x = (errDistance * Math.cos(errAngle * (Math.PI / 180))) * speed;
        // caps the X velocity to the maximum allowed velocity
        if (x > MAX_DRIVE) {
          x = MAX_DRIVE;
        }
        else if (x < -MAX_DRIVE) {
          x = -MAX_DRIVE;
        }
        return x;
    }
    public static double getDriveY(double thetaX, double thetaY, double distance, double DESIRED_TARGET_DISTANCE, double DESIRED_HEADING){ 
        // These numbers must be tuned for your Robot!  Be careful!
        final double DRIVE_V = 0.4;                    // How hard to drive fwd toward the target
        final double MAX_DRIVE = 0.4;    // maximum allowed drive velocity
        
        // calculates horizontal distance to target 
        m_HorizontalDistance = distance * Math.cos(thetaY * (Math.PI / 180));

        // if distance within 0.15 of desired distance set error to 0
        double errDistance = m_HorizontalDistance - DESIRED_TARGET_DISTANCE;
        if (Math.abs(errDistance) < 100) {
          errDistance = 0;
        }

        // if angle within 2 degrees of desired angle set error to 0
        double errAngle = DESIRED_HEADING - thetaX;
        if (Math.abs(errAngle) < 2) {
          errAngle = 0;
        }
        // calculates robot movment velocity baised on current distance from target
        double speed = Math.abs(DRIVE_V * errDistance);

        // calculates theoretical Y velocity 
        double y = (errDistance * Math.sin(errAngle * (Math.PI / 180))) * speed;
        // caps the Y velocity to the maximum allowed velocity
        if (y > MAX_DRIVE) {
          y = MAX_DRIVE;
        }
        else if (y < -MAX_DRIVE) {
          y = -MAX_DRIVE;
        }
        return y;
    }
    public static double getDriveRot(double thetaX, double DESIRED_HEADING){          
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_V = 0.01;                  // How hard to turn toward the target
        final double MAX_TURN = 0.1;    // Maximum allowed rotational velocity
 
        // if angle within 2 degrees of desired angle set error to 0
        double errAngle = DESIRED_HEADING - thetaX;
        if (Math.abs(errAngle) < 2) {
          errAngle = 0;
        }
        
        // calculates theoretical rotational velocity
        double rotate = (errAngle * STEER_V);
        // caps the rotaional velocity to the maximum allowed velocity
        if (rotate > MAX_TURN) {
            rotate = MAX_TURN;
        }
        else if (rotate < -MAX_TURN) {
            rotate = -MAX_TURN;
        }
        return rotate;
    }
        
}