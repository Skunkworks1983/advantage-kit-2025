// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.utils.constants.CollectorConstants;
import frc.robot.utils.constants.CurrentLimitConstants;
import frc.robot.utils.constants.EndEffectorSetpointConstants;
import frc.robot.utils.constants.EndEffectorToSetpointConstants;
import frc.robot.utils.constants.OIConstants.OI.IDs.Joysticks;
import java.util.function.Supplier;

public class Collector extends SubsystemBase {

  private TalonFX rightMotor;
  private TalonFX leftMotor;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private double lastSpeed;

  // Neither of these smart PIDs are 'used' after they are constructed because the
  // PID controller is built into the motor (we don't have to call .calculate like
  // we
  // do with the PIDController class).
  @SuppressWarnings("unused")
  private SmartPIDControllerTalonFX rightMotorController;

  private DigitalInput beambreak;

  double collectorSetpoint;
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
  double lastThrottle = 0.0;

  /** Creates a new Collector. */
  public Collector() {
    rightMotor = new TalonFX(CollectorConstants.IDs.RIGHT_MOTOR, "Collector 2025");
    leftMotor = new TalonFX(CollectorConstants.IDs.LEFT_MOTOR, "Collector 2025");

    setDefaultCommand(holdPositionCommand());

    TalonFXConfiguration talonConfigCollectorMotor = new TalonFXConfiguration();
    talonConfigCollectorMotor.CurrentLimits = CurrentLimitConstants.KRAKEN_CURRENT_LIMIT_CONFIG;

    talonConfigCollectorMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotor.getConfigurator().apply(talonConfigCollectorMotor);
    leftMotor.getConfigurator().apply(talonConfigCollectorMotor);

    // Dual Pids with two slots, one for velocity and one for position
    rightMotorController =
        new SmartPIDControllerTalonFX(
            CollectorConstants.VelocityControlMode.KP, // VELOCITY
            CollectorConstants.VelocityControlMode.KI,
            CollectorConstants.VelocityControlMode.KD,
            CollectorConstants.VelocityControlMode.KF,
            CollectorConstants.VelocityControlMode.KV,
            CollectorConstants.VelocityControlMode.KA,
            CollectorConstants.VelocityControlMode.KS,
            "right motor",
            CollectorConstants.SMART_PID_ENABLED,
            rightMotor);
    rightMotorController.AddSlot1Configs(
        CollectorConstants.PositionControlMode.KP, // POSITION
        CollectorConstants.PositionControlMode.KI,
        CollectorConstants.PositionControlMode.KD,
        CollectorConstants.PositionControlMode.KF,
        CollectorConstants.PositionControlMode.KV,
        CollectorConstants.PositionControlMode.KA,
        CollectorConstants.PositionControlMode.KS);

    leftMotor.setControl(new Follower(CollectorConstants.IDs.RIGHT_MOTOR, true));

    beambreak = new DigitalInput(CollectorConstants.IDs.DIGITAL_INPUT_CHANNEL);
  }

  private void setCollectorThrottle(double throttle) {

    if (throttle != lastThrottle) {
      System.out.println("Set collector throttle: " + throttle);
      rightMotor.setControl(dutyCycleOut.withOutput(throttle).withEnableFOC(true));
      lastThrottle = throttle;
    }
  }

  // meters per sec
  private void setCollectorSpeeds(double speed) {
    // Resetting last throttle
    lastThrottle = 0;
    if (speed != lastSpeed) {
      rightMotor.setControl(
          velocityVoltage
              .withVelocity(speed * CollectorConstants.COLLECTOR_ROTATIONS_PER_METER)
              .withEnableFOC(true)
              .withSlot(0));
      ConditionalSmartDashboard.putNumber("Collector/Right speed", speed);
    }
    lastSpeed = speed;
  }

  @Override
  public void periodic() {
    rightMotorController.updatePID();

    ConditionalSmartDashboard.putNumber(
        "Collector/Right motor current", rightMotor.getSupplyCurrent().getValueAsDouble());
    ConditionalSmartDashboard.putNumber(
        "Collector/Left motor current", leftMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Collector/Beambreak collector", !beambreak.get());
  }

  public double getLeftMotorVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public double getRightMotorVelocity() {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  public double getRightMotorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  public double getLeftMotorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public boolean isHoldingCoral() {
    return !beambreak.get();
  }

  public void setCollectorSetPoint(double newRightSetpoint) {
    rightMotor.setControl(positionVoltage.withPosition(newRightSetpoint).withSlot(1));
  }

  public Command rotateCoralCommand() {
    return runEnd(
        () -> {
          setCollectorSpeeds(CollectorConstants.Speeds.CORAL_INTAKE_SLOW_SPEED);
          ConditionalSmartDashboard.putNumber(
              "Collector/Right collector current speed", getRightMotorVelocity());
          ConditionalSmartDashboard.putNumber(
              "Collector/Left collector current speed", getLeftMotorVelocity());
        },
        () -> {
          setCollectorSpeeds(0);
        });
  }

  // Rrue if you want it to stop the motor when the command ends
  // it should almost always be true unless there will be a following command right after that will
  // end it
  public Command intakeCoralCommand(
      boolean stopOnEnd, Supplier<EndEffectorToSetpointConstants> endEffectorSetpoint) {
    int endCount[] = {0}; // This value needs to be effectivly final
    return startEnd(
            () -> {
              if (endEffectorSetpoint.get() == EndEffectorSetpointConstants.CORAL_GROUND) {
                setCollectorSpeeds(CollectorConstants.Speeds.CORAL_INTAKE_FAST_SPEED);
              } else {
                setCollectorSpeeds(CollectorConstants.Speeds.CORAL_INTAKE_SLOW_SPEED);
              }
            },
            () -> {
              if (stopOnEnd) {
                setCollectorSpeeds(0);
              }
            })
        .beforeStarting(
            () -> {
              endCount[0] = 0;
            })
        .until(
            () -> {
              ConditionalSmartDashboard.putNumber(
                  "Collector/Amp cut off right", rightMotor.getSupplyCurrent().getValueAsDouble());
              ConditionalSmartDashboard.putNumber(
                  "Collector/Amp cut off left", leftMotor.getSupplyCurrent().getValueAsDouble());
              if (!beambreak.get()) {
                endCount[0]++;
              } else {
                endCount[0] = 0;
              }
              return endCount[0] >= CollectorConstants.END_COUNT_TICK_COUNTER_CORAL;
            });
  }

  public Command expelCoralCommand(
      boolean stopOnEnd, Supplier<EndEffectorToSetpointConstants> endEffectorSetpoint) {
    return runEnd(
        () -> {
          if (endEffectorSetpoint.get().equals(EndEffectorSetpointConstants.CORAL_L4)) {
            setCollectorSpeeds(CollectorConstants.Speeds.CORAL_EXPEL_L4_SPEED);
          } else if (endEffectorSetpoint.get().equals(EndEffectorSetpointConstants.CORAL_L1)) {
            setCollectorSpeeds(
                -CollectorConstants.Speeds
                    .CORAL_EXPEL_SLOW_SPEED); // Changed to negative because coral now shoots out
            // opposite direction
          } else {
            setCollectorSpeeds(CollectorConstants.Speeds.CORAL_EXPEL_FAST_SPEED);
          }
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0);
          }
        });
  }

  public Command expelCoralCommandWithSensor(
    boolean stopOnEnd, Supplier<EndEffectorToSetpointConstants> endEffectorSetpoint) {
  return runEnd(
      () -> {
        if (endEffectorSetpoint.get().equals(EndEffectorSetpointConstants.CORAL_L4)) {
          setCollectorSpeeds(CollectorConstants.Speeds.CORAL_EXPEL_L4_SPEED);
        } else if (endEffectorSetpoint.get().equals(EndEffectorSetpointConstants.CORAL_L1)) {
          setCollectorSpeeds(
              -CollectorConstants.Speeds
                  .CORAL_EXPEL_SLOW_SPEED); // Changed to negative because coral now shoots out
          // opposite direction
        } else {
          setCollectorSpeeds(CollectorConstants.Speeds.CORAL_EXPEL_FAST_SPEED);
        }
      },
      () -> {
        if (stopOnEnd) {
          setCollectorSpeeds(0);
        }
      }).until(() -> {
        return !isHoldingCoral();
      });
}

  public Command holdPositionCommand() {
    Trigger algaeToggle =
        new JoystickButton(
            new Joystick(Joysticks.BUTTON_STICK_ID),
            frc.robot.utils.constants.OIConstants.OI.IDs.Buttons.ALGAE_TOGGLE);

    return startEnd(
            () -> {
              if (algaeToggle.getAsBoolean()) {
                setCollectorThrottle(CollectorConstants.Speeds.ALGAE_INTAKE_SPEED_SLOW);
              } else {
                setCollectorSetPoint(getRightMotorPosition());
              }
            },
            () -> {})
        .until(
            () -> {
              return algaeToggle.getAsBoolean();
            })
        .repeatedly();
  }

  public Command intakeAlgaeCommand(
      boolean stopOnEnd, Supplier<EndEffectorToSetpointConstants> endEffectorSetpoint) {
    int endCount[] = {0}; // This value needs to be effectivly final
    return runEnd(
            () -> {
              if (endEffectorSetpoint.get() == EndEffectorSetpointConstants.ALGAE_GROUND) {
                setCollectorThrottle(CollectorConstants.Speeds.ALGAE_INTAKE_SPEED_FAST);
              } else {
                setCollectorThrottle(CollectorConstants.Speeds.ALGAE_INTAKE_SPEED_SLOW);
              }
            },
            () -> {
              if (stopOnEnd) {
                setCollectorSpeeds(0);
              }
            })
        .beforeStarting(
            () -> {
              lastThrottle = 0.0;
              endCount[0] = 0;
            })
        .until(
            () -> {
              if (rightMotor.getSupplyCurrent().getValueAsDouble()
                  >= CollectorConstants.ALGAE_AMP_CUT_OFF) {
                endCount[0]++;
              } else {
                endCount[0] = 0;
              }
              return endCount[0] > CollectorConstants.END_COUNT_TICK_COUNTER_ALGAE;
            });
  }

  public Command expelAlgaeCommand(boolean stopOnEnd) {
    return runEnd(
        () -> {
          setCollectorSpeeds(CollectorConstants.Speeds.ALGAE_EXPEL_SPEED);
        },
        () -> {
          if (stopOnEnd) {
            setCollectorSpeeds(0);
          }
        });
  }
}
