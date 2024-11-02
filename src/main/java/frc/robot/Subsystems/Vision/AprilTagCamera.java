package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionFrame;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** Generic class for a 2D Apriltag Tracking Camer */
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

  /** Function to update vision data. Should only be called once per robot loop. */
  public void updateVisionData() {

    // Get the latest data from the coprocessor.
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // Only process if we have visible targets
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // Create a variable for logging visible tags
        int index = 0;
        corners = new Translation2d[result.targets.size() * 4];

        // Get a list of all visible apriltags
        List<PhotonTrackedTarget> tags = result.getTargets();

        // Loop through the tags
        for (PhotonTrackedTarget tag : tags) {

          // Add corner positions to an array for logging purposes
          for (TargetCorner corner : tag.getDetectedCorners()) {
            Translation2d cornerTranslation = new Translation2d(corner.x, corner.y);
            corners[index] = cornerTranslation;
            index++;
          }

          // Check the current tag ID
          int tagID = tag.getFiducialId();

          // Only update output data if we can see the center speaker tag
          // 4 is Red, 7 is Blue
          if (tagID == 4 || tagID == 7) {
            hasTarget = true;
            tX = tag.getYaw();
            tY = tag.getPitch();
          }
        }
      }
    } else {
      // Return default values if no tags are visible
      corners = new Translation2d[0];
      tX = 0;
      tY = 0;
      hasTarget = false;
    }

    // Log vision data
    Logger.recordOutput("Vision/" + camera.getName() + "/tX", tX);
    Logger.recordOutput("Vision/" + camera.getName() + "/tY", tY);
    Logger.recordOutput("Vision/" + camera.getName() + "/Corners", corners);
    Logger.recordOutput("Vision/" + camera.getName() + "/hasTarget", hasTarget);
  }

  // Function for returning the latest vision data.
  public VisionFrame getVisionFrame() {
    return new VisionFrame(tX, tY, hasTarget);
  }
}
