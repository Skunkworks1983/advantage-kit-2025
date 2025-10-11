// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerCANSparkMax;
import frc.robot.utils.constants.ClimberConstants;
import frc.robot.utils.constants.FunnelConstants;

public class Funnel extends SubsystemBase {

  SparkMax pivotMotor;

  private SmartPIDControllerCANSparkMax pivotMotorSpeedController;

  double setpoint;

  public Funnel() {
    pivotMotor = new SparkMax(FunnelConstants.IDs.PIVOT_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .p(FunnelConstants.PIDs.FUNNEL_KP)
        .i(FunnelConstants.PIDs.FUNNEL_KI)
        .d(FunnelConstants.PIDs.FUNNEL_KD);
    pivotMotor.getEncoder().setPosition(0);

    // NOTE: we are intentionally not setting a current limit because this subsystem
    // is rarely used and will likely not exceed a reasonable current.

    pivotMotorSpeedController =
        new frc.robot.utils.PIDControllers.SmartPIDControllerCANSparkMax(
            FunnelConstants.PIDs.FUNNEL_KP,
            FunnelConstants.PIDs.FUNNEL_KI,
            FunnelConstants.PIDs.FUNNEL_KD,
            FunnelConstants.PIDs.FUNNEL_KF,
            "Funnel Pivot Motor",
            FunnelConstants.PIDs.FUNNEL_SMARTPID_ACTIVE,
            pivotMotor);
  }

  @Override
  public void periodic() {
    pivotMotorSpeedController.updatePID();
  }

  public double getPos() {
    return pivotMotor.getEncoder().getPosition();
  }

  public double getSetpoint() {
    return setpoint;
  }

  public boolean isMotorConnected() {
    return pivotMotor.getFirmwareVersion() != 0;
  }

  public double getCurrent() {
    return pivotMotor.getOutputCurrent();
  }

  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  public boolean isAtSetpoint() {
    return approxEquals(getPos(), getSetpoint(), ClimberConstants.CLIMBER_TOLERANCE);
  }

  public double getVelocity() {
    return pivotMotor.getEncoder().getVelocity();
  }

  public void setFunnelSetPoint(double revs) {
    setpoint = revs;
    SparkClosedLoopController FunnelLoopController = pivotMotor.getClosedLoopController();
    FunnelLoopController.setReference(getSetpoint(), ControlType.kPosition);
  }
}
