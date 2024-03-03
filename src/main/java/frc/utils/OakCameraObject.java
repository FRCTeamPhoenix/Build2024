package frc.utils;

import java.lang.Math;

public class OakCameraObject {

    private final double xAngle;
    private final double yAngle;
    private final double area;
    private final double cameraDistance;
    private final double confedence;
    private final String type;
    private final double horizontalDistance;

    public OakCameraObject(String objectData) {
        String[] splitData = objectData.split(", ");
        this.xAngle = Float.parseFloat(splitData[0]);
        this.yAngle = Float.parseFloat(splitData[1]);
        this.area = Float.parseFloat(splitData[2]);
        this.cameraDistance = Float.parseFloat(splitData[3]);
        this.confedence = Float.parseFloat(splitData[4]);
        this.type = splitData[5];
        this.horizontalDistance = cameraDistance * Math.sin(Math.toRadians(90 + yAngle));
    }

    public double getXAngle() {
        return this.xAngle;
    }

    public double getYAngle() {
        return this.yAngle;
    }

    public double getArea() {
        return this.area;
    }

    public double getCameraDistance() {
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