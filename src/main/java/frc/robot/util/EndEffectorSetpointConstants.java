package frc.robot.util;

public class EndEffectorSetpointConstants {
  public final double elevatorSetpoint;
  public final double wristSetpoint;
  public final double stowSetpoint;
  public EndEffectorSetpointConstants(double elevatorSetpoint, double wristSetpoint, double stowSetpoint) {
    this.elevatorSetpoint = elevatorSetpoint;
    this.wristSetpoint = wristSetpoint;
    this.stowSetpoint = stowSetpoint;
  }
}
