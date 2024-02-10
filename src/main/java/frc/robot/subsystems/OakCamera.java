package frc.robot.subsystems;
import java.util.*;

import java.util.List;

import java.util.ArrayList;

//import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.OakCameraObject;
import frc.utils.CameraDriveUtil;

public class OakCamera extends SubsystemBase {

  public static List<OakCameraObject> extractOakData() {
    List<OakCameraObject> cameraObjects = new ArrayList<>();
    String[] fallback = new String[0];
    for (int i = 0 ; i < 3 ; i++) {
      String[] cameraPredictions = NetworkTableInstance.getDefault().getTable("oakCamera").getEntry("cameraItems_" + String.valueOf(i)).getStringArray(fallback);
      for (int objectNumber = 0; objectNumber < cameraPredictions.length; objectNumber++ ) {
        cameraObjects.add(new OakCameraObject(cameraPredictions[objectNumber]));
      }
    }
    return cameraObjects;
  }

  public static boolean hasValidTarget() {
    if (extractOakData().size() != 0) {
      return true;
    }
    else {
      return false;
    }
  }

  public OakCameraObject findClosestNote() {
    List<OakCameraObject> cameraObjects = extractOakData();
    double minimumDistance = Double.MAX_VALUE;
    OakCameraObject closestNote = null;
    // loop through ever object the cammera detects
    for (OakCameraObject objectInstance : cameraObjects) {
      //filter out non notes
      if (objectInstance.getType() != "note") {
        continue;
      }
      //filter out notes that are too far
      if (objectInstance.getHorizontalDistance() >= minimumDistance) {
        continue;
      }
      //update closest notes
      minimumDistance = objectInstance.getHorizontalDistance();
      closestNote = objectInstance;
    }
    // return the closest notes
    return closestNote;
  }

  @Override
  public void periodic() {

  }
}

