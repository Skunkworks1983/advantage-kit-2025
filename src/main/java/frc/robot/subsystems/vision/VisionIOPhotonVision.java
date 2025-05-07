package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {

  private final String cameraName;
  private final Transform3d robotToCamera;
  private final PhotonCamera camera;
  private final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.camera = new PhotonCamera(cameraName);
  }

  @Override
  public VisionIOData getLatestData() {
    VisionIOData latestData = new VisionIOData();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

      if (result.getMultiTagResult().isPresent()) {
        MultiTargetPNPResult multitagResult = result.getMultiTagResult().get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d estimatedRobotPose =
            new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalTagDistance = 0.0;
        for (PhotonTrackedTarget target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        latestData.poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                estimatedRobotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size()));

      } else if (!result.targets.isEmpty()) {
        PhotonTrackedTarget target = result.targets.get(0);
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.fiducialId);

        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());

          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d estimatedRobotPose =
              new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          latestData.poseObservations.add(
              new PoseObservation(
                  // If the timestamp is in the future, then use the FPGA timestamp instead.
                  // Future timestamps can break the pose estimator.
                  Math.min(result.getTimestampSeconds(), Timer.getFPGATimestamp()),
                  estimatedRobotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm()));
        }
      }
    }

    return latestData;
  }

  @Override
  public String getName() {
    return cameraName;
  }
}
