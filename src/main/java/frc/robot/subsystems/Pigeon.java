package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;

abstract class PigeonBase {
    abstract Pigeon2 getPigeon2();
    abstract WPI_PigeonIMU getPigeon();
    abstract void setupPigeon(int deviceID, String canbus);
    abstract double getYaw();
    abstract double getPitch();
    abstract double getRoll();
    abstract double getRate();
    abstract double getAngle();
    abstract void reset();
}

class IMU_Pigeon2 extends PigeonBase{
    public Pigeon2 pigeon = new Pigeon2(0);
    void setupPigeon(int deviceID, String canbus){pigeon = new Pigeon2(deviceID, canbus);}
    double getYaw(){return pigeon.getYaw().getValueAsDouble();}
    double getPitch(){return pigeon.getPitch().getValueAsDouble();}
    double getRoll(){return pigeon.getRoll().getValueAsDouble();}
    double getRate(){return pigeon.getRate();}
    double getAngle(){return pigeon.getAngle();}
    void reset(){pigeon.reset();}
    Pigeon2 getPigeon2(){return pigeon;}
    WPI_PigeonIMU getPigeon(){return null;}
}

class IMU_Pigeon extends PigeonBase{
    public WPI_PigeonIMU pigeon = new WPI_PigeonIMU(0);
    void setupPigeon(int deviceID, String canbus){pigeon = new WPI_PigeonIMU(deviceID);}
    double getYaw(){return pigeon.getYaw();}
    double getPitch(){return pigeon.getPitch();}
    double getRoll(){return pigeon.getRoll();}
    double getRate(){return pigeon.getRate();}
    double getAngle(){return pigeon.getAngle();}
    void reset(){pigeon.reset();}
    WPI_PigeonIMU getPigeon(){return pigeon;}
    Pigeon2 getPigeon2(){return null;}
}
