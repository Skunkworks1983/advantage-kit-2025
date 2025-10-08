// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;
import frc.robot.subsystems.vision.constants.VisionIOConstants;
import java.util.LinkedList;
import java.util.List;

/**
 * A subsystem that takes a VisionConsumer and any number of VisionIOConstants. Calls accept() on
 * the VisionConsumer in periodic() to send latest vision measurements. VisionConsumer is intended
 * to be used by a SwerveDrivePoseEstimator in Drivebase.
 */
public class Vision extends SubsystemBase {

  private VisionConsumer consumer;
  private LinkedList<VisionIO> ios = new LinkedList<VisionIO>();
  VisionIOInputs[] inputs;
  private List<Field2d> field2ds = new LinkedList<Field2d>();

  public Vision(VisionConsumer consumer, VisionIOConstants... ioConstants) {
    this.consumer = consumer;

    System.out.println("VISION CONSTRUCTOR RUNNING");

    for (VisionIOConstants constants : ioConstants) {
      try {
        VisionIO initializedIO = constants.init();
        ios.add(initializedIO);
        Field2d field2d = new Field2d();
        SmartDashboard.putData(initializedIO.getName() + " Odometry", field2d);
        System.out.println("PUTTING VISION FIELD2D TO SMARTDASHBOARD");
        field2ds.add(field2d);
      } catch (Exception e) {
        System.err.println(
            "A vision i/o failed to initialize. Double-check that the camera is plugged in"
                + "and the camera is working.");
        System.out.println("FAILED TO INIT CAMERA");
        e.printStackTrace();
        continue;
      }

      this.inputs = new VisionIOInputs[ios.size()];

      for (int i = 0; i < ios.size(); i++) {
        this.inputs[i] = new VisionIOInputs();
      }
    }
  }

  @Override
  public void periodic() {

    for (int i = 0; i < ios.size(); i++) {
      ios.get(i).updateInputs(inputs[i]);
    }

    for (int a = 0; a < inputs.length; a++) {
      VisionIOInputs input = inputs[a];
      for (int b = 0; b < input.measurements.length; b++) {
        VisionMeasurement measurement = input.measurements[b];
        // field2ds.get(i).setRobotPose(measurement.estimatedPose().toPose2d());
        consumer.accept(
            measurement.estimatedPose().toPose2d(), measurement.timestamp(), measurement.stdDevs());
      }
      // for (VisionMeasurement measurement : input.measurements) {
      //   consumer.accept(
      //       measurement.estimatedPose().toPose2d(), measurement.timestamp(),
      // measurement.stdDevs());
      // }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(Pose2d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
