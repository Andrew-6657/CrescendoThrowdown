package frc.robot.Subsystems.Vision;

import frc.robot.Constants.VisionFrame;

public class Vision {
  private static AprilTagCamera camera = new AprilTagCamera("OV2311_Back");

  public VisionFrame getVisionFrame() {
    return camera.getVisionFrame();
  }

  public void updateVisionData() {
    camera.updateTargetData();
  }
}
