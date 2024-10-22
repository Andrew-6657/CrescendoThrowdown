package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionFrame;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class AprilTagCamera {

  private PhotonCamera camera;

  private double tX = 0;
  private double tY = 0;
  private Translation2d[] corners = {};
  boolean hasTarget = false;

  public AprilTagCamera(String name) {
    camera = new PhotonCamera(name);
    camera.setDriverMode(false);
  }

  public void updateTargetData() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {

      int index = 0;
      corners = new Translation2d[result.targets.size() * 4];

      List<PhotonTrackedTarget> tags = result.getTargets();

      for (PhotonTrackedTarget tag : tags) {

        for (TargetCorner corner : tag.getDetectedCorners()) {
          Translation2d cornerTranslation = new Translation2d(corner.x, corner.y);
          corners[index] = cornerTranslation;
          index++;
        }

        int tagID = tag.getFiducialId();

        if (tagID == 4 || tagID == 7) { // 4 is Red, 7 is Blue
          hasTarget = true;
          tX = tag.getYaw();
          tY = tag.getPitch();
        }
      }
    } else {
      corners = new Translation2d[0];
      tX = 0;
      tY = 0;
      hasTarget = false;
    }

    Logger.recordOutput("Vision/" + camera.getName() + "/tX", tX);
    Logger.recordOutput("Vision/" + camera.getName() + "/tY", tY);
    Logger.recordOutput("Vision/" + camera.getName() + "/Corners", corners);
    Logger.recordOutput("Vision/" + camera.getName() + "/hasTarget", hasTarget);
  }

  public VisionFrame getVisionFrame() {
    return new VisionFrame(tX, tY, hasTarget);
  }
}
