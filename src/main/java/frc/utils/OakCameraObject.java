import java.lang.Math;

public class oakCameraObject {

    private final double xAngle;
    private final double yAngle;
    private final int area;
    private final int distanceCamera;
    private final String type;
    private final double distance;

    public oakCameraObject(String objectDataArray) {
        String[] objectData = objectData.split(", ");
        this.xAngle = Math.toRadians(Float.parseFloat(objectData[0]));
        this.yAngle = Math.toRadians(Float.parseFloat(objectData[1]));
        this.area = Integer.parseInt(objectData[2]);
        this.distanceCamera = Integer.parseInt(objectData[3]);
        this.type = objectData[4];
        this.distance = distanceCamera * sin(yAngle - 90);
    }

    public returnData(String request) {
        switch (request) {
            case "xAngle":
                return this.xAngle;
                break;
            case "yAngle":
                return this.yAngle;
                break;
            case "area":
                return this.area;
                break;
            case "distanceCamera":
                return this.distanceCamera;
                break;
            case "type":
                return this.type;
                break;
            case "distance":
                return this.distance;
                break;
        }
    }
}