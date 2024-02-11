package frc.utils;
import java.lang.Math;

public class OakCameraObject {

    private final double xAngle;
    private final double yAngle;
    private final int area;
    private final int cameraDistance;
    private final double confedence;
    private final String type;
    private final double horizontalDistance;

    public OakCameraObject(String objectData) {
        String[] splitData = objectData.split(", ");
        this.xAngle = Math.toRadians(Float.parseFloat(splitData[0]));
        this.yAngle = Math.toRadians(Float.parseFloat(splitData[1]));
        this.area = Integer.parseInt(splitData[2]);
        this.cameraDistance = Integer.parseInt(splitData[3]);
        this.confedence = Float.parseFloat(splitData[4]);
        this.type = splitData[5];
        this.horizontalDistance = cameraDistance * Math.sin(yAngle - 90);
    }

    public double getXAngle() {
        return this.xAngle;
    }
    public double getYAngle() {
        return this.yAngle;
    }
    public int getArea() {
        return this.area;    
    }
    public int getCameraDistance() {
        return this.cameraDistance;    
    }
    public Double getConfedence() {
        return this.confedence;  
    }
    public String getType() {
        return this.type;   
    }
    public Double getHorizontalDistance() {
        return this.horizontalDistance;  
    }


    
}