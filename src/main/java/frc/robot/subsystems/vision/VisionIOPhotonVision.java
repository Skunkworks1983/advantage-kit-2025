package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {

  private final String cameraName;
  private final Transform3d robotToCamera;
  final PhotonCamera camera;
  final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private final double kAngularStdDev = 15.0;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.camera = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    List<VisionMeasurement> measurements = new LinkedList<VisionMeasurement>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

      for (PhotonTrackedTarget target : result.getTargets()) {
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {
          Pose3d fieldToTarget = tagPose.get();
          Transform3d cameraToTarget = target.getBestCameraToTarget();
          Pose3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Pose3d robotPose = fieldToCamera.plus(robotToCamera.inverse());
          double distToTag = cameraToTarget.getTranslation().getNorm();

          // Reject the pose if it's not within the field boundries.
          boolean rejectPose =
              robotPose.getX() < 0.0
                  || robotPose.getX() > aprilTagLayout.getFieldLength()
                  || robotPose.getY() < 0.0
                  || robotPose.getY() > aprilTagLayout.getFieldWidth();

          if (rejectPose) {
            continue;
          }

          // Quadratic relationship between std dev and distance to the tag.
          double translationalStdDev = (0.0329)*Math.pow(distToTag, 2) + (-0.0222)*distToTag + (0.0048);;

          measurements.add(
              new VisionMeasurement(
                  robotPose,
                  result.getTimestampSeconds(),
                  VecBuilder.fill(translationalStdDev, translationalStdDev, kAngularStdDev)));
        }
      }
    }

    inputs.measurements = new VisionMeasurement[measurements.size()];

    for (int i = 0; i < measurements.size(); i++) {
      inputs.measurements[i] = measurements.get(i);
    }
  }

  @Override
  public String getName() {
    return cameraName;
  }
}
