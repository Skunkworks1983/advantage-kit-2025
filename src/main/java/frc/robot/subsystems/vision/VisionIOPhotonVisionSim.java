// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Currently, simulation for vision can't be used because no simulation software exists for the drivebase. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonVisionSim(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {

    super(cameraName, robotToCamera);

    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
