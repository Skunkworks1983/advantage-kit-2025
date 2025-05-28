// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.utils.constants.ElevatorConstants;
import frc.robot.utils.constants.EndEffectorToSetpointConstants;

// All positions are stored in meters, all velocities are in meters/seconds,
// all accelerations are stored in meters/second/second.
public class Elevator extends SubsystemBase {

  public TalonFX motorRight = new TalonFX(ElevatorConstants.MOTOR_RIGHT_ID);
  private TalonFX motorLeft = new TalonFX(ElevatorConstants.MOTOR_LEFT_ID);

  // We do not currently have working limit switches on the wrist
  private DigitalInput bottomLimitSwitch =
      new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_ID);
  // private DigitalInput topLimitSwitch = new DigitalInput(Constants.Elevator.TOP_LIMIT_SWITCH_ID);

  private double finalTargetPosition;

  private SmartPIDControllerTalonFX smartPIDController;

  private EndEffectorToSetpointConstants endEffectorSetpoint =
      frc.robot.utils.constants.EndEffectorSetpointConstants.CORAL_STOW;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits =
        frc.robot.utils.constants.CurrentLimitConstants.KRAKEN_CURRENT_LIMIT_CONFIG;
    motorRight.getConfigurator().apply(config);
    motorLeft.getConfigurator().apply(config);

    smartPIDController =
        new SmartPIDControllerTalonFX(
            ElevatorConstants.PIDs.ELEVATOR_kP,
            ElevatorConstants.PIDs.ELEVATOR_kI,
            ElevatorConstants.PIDs.ELEVATOR_kD,
            ElevatorConstants.PIDs.ELEVATOR_kF,
            ElevatorConstants.PIDs.ELEVATOR_kV,
            ElevatorConstants.PIDs.ELEVATOR_kA,
            ElevatorConstants.PIDs.ELEVATOR_kS,
            "Elevator",
            ElevatorConstants.PIDs.SMART_PID_ENABLED,
            motorRight);

    motorRight.setNeutralMode(NeutralModeValue.Brake);
    motorLeft.setNeutralMode(NeutralModeValue.Brake);

    // True means that the motor will be spinning opposite of the one it is following
    motorLeft.setControl(new Follower(ElevatorConstants.MOTOR_RIGHT_ID, true));
  }

  @Override
  public void periodic() {
    smartPIDController.updatePID();

    // Setposition counts as a config update, try and do this sparingly
    if (getBottomLimitSwitch() && Math.abs(motorRight.getPosition().getValueAsDouble()) > .001) {
      motorRight.setPosition(0.0);
    }
    // else if (getTopLimitSwitch()) {
    //   motorRight.setPosition(Constants.Elevator.MAX_HEIGHT_CARRIAGE *
    // Constants.Elevator.METERS_TO_MOTOR_ROTATIONS);
    // }
    putInfoSmartDashboard();
  }

  // Reminder: all positions are measured in meters
  public double getElevatorPosition() {
    return motorRight.getPosition().getValueAsDouble()
        * ElevatorConstants.MOTOR_ROTATIONS_TO_METERS;
  }

  // Reminder: all velocities are measured in meters/second
  public double getElevatorVelocity() {
    return motorRight.getVelocity().getValueAsDouble()
        * ElevatorConstants.MOTOR_ROTATIONS_TO_METERS;
  }

  // Inverted because limit switches return true until tripped
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  // Inverted because limit switches return true until tripped
  // public boolean getTopLimitSwitch() {
  //   !topLimitSwitch.get();
  // }

  public void setMotorTrapezoidProfileSafe(double position, double velocity) {
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    positionVoltage.Position = position;
    positionVoltage.Velocity = velocity;
    logTargetPosition(position);
    logTargetVelocity(velocity);

    motorRight.setControl(
        positionVoltage
            //  .withLimitForwardMotion(getTopLimitSwitch())
            //  .withLimitReverseMotion(getBottomLimitSwitch())
            .withEnableFOC(true));
  }

  public boolean isAtSetpoint(double targetPosition) {
    return Math.abs(getElevatorPosition() - targetPosition)
        < ElevatorConstants.TOLORENCE_METERS_FOR_SETPOINT;
  }

  public void logTargetPosition(double targetPosition) {
    ConditionalSmartDashboard.putNumber("Elevator/Desired position in meters", targetPosition);

    ConditionalSmartDashboard.putNumber(
        "Elevator/Desired position in rotations",
        targetPosition * ElevatorConstants.METERS_TO_MOTOR_ROTATIONS);

    ConditionalSmartDashboard.putNumber(
        "Elevator/Error in rotations",
        targetPosition - motorRight.getPosition().getValueAsDouble());
  }

  public void setFinalPosition(double finalTargetPosition) {
    this.finalTargetPosition = finalTargetPosition;
  }

  public void logTargetVelocity(double targetVelocity) {
    ConditionalSmartDashboard.putNumber("Elevator/Desired velocity in mps", targetVelocity);
  }

  public void putInfoSmartDashboard() {
    double currentPos = motorRight.getPosition().getValueAsDouble();

    ConditionalSmartDashboard.putNumber(
        "Elevator/Actual velocity in mps", motorRight.getVelocity().getValueAsDouble());
    ConditionalSmartDashboard.putNumber(
        "Elevator/Actual position in meters",
        currentPos * ElevatorConstants.MOTOR_ROTATIONS_TO_METERS);
    ConditionalSmartDashboard.putNumber("Elevator/Actual position in rotations", currentPos);
    ConditionalSmartDashboard.putBoolean("Elevator/Bottom limit switch", getBottomLimitSwitch());
    // ConditionalSmartDashboard.putBoolean("Elevator/Top limit switch", getTopLimitSwitch());
  }

  public void setElevatorMotorControl(PositionVoltage setElevatorMotorControl) {
    motorRight.setControl(
        setElevatorMotorControl
            //  .withLimitForwardMotion(getTopLimitSwitch())
            //  .withLimitReverseMotion(getBottomLimitSwitch())
            .withEnableFOC(true));
  }

  public void setSpeeds(double speed) {
    motorRight.setControl(new DutyCycleOut(speed));
  }

  public void setEndEffectorSetpoint(EndEffectorToSetpointConstants endEffectorSetpoint) {
    this.endEffectorSetpoint = endEffectorSetpoint;
  }

  public EndEffectorToSetpointConstants getEndEffectorSetpoint() {
    return endEffectorSetpoint;
  }
}
