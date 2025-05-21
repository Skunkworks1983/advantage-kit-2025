// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.constants;

public class EndEffectorToSetpointConstants {
  public final double elevatorSetpoint;
  public final double wristSetpoint;
  public final double stowSetpoint;

  public EndEffectorToSetpointConstants(
      double elevatorSetpoint, double wristSetpoint, double stowSetpoint) {
    this.elevatorSetpoint = elevatorSetpoint;
    this.wristSetpoint = wristSetpoint;
    this.stowSetpoint = stowSetpoint;
  }
}
