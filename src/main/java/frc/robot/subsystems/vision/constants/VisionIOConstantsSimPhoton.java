// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.Supplier;

/** Add your docs here. */
public class VisionIOConstantsSimPhoton implements VisionIOConstants {

  private final String cameraName;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOConstantsSimPhoton(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.cameraName = cameraName;
    this.robotToCamera = robotToCamera;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public VisionIO init() {
    return new VisionIOPhotonVisionSim(cameraName, robotToCamera, poseSupplier);
  }
}
