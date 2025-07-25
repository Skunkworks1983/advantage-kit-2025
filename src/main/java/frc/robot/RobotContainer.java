// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutomatedLidarScoring;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.funnel.MoveFunnelToSetpoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.constants.VisionConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.constants.EndEffectorSetpointConstants;
import frc.robot.utils.constants.FunnelConstants;
import frc.robot.utils.constants.OIConstants;
import frc.robot.utils.constants.OIConstants.OI;
import frc.robot.utils.constants.OIConstants.OI.IDs.Joysticks;
import frc.robot.utils.constants.SimConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  frc.robot.subsystems.elevator.Elevator elevator;
  frc.robot.subsystems.wrist.Wrist wrist;
  frc.robot.subsystems.collector.Collector collector;

  // Controller
  // private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick translationJoystick = new Joystick(Joysticks.TRANSLATION_JOYSTICK_ID);
  private final Joystick rotationJoystick = new Joystick(Joysticks.ROTATION_JOYSTICK_ID);
  private Joystick buttonJoystick = new Joystick(Joysticks.BUTTON_STICK_ID);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        elevator = new Elevator();
        wrist = new Wrist();
        collector = new Collector();

        Vision vision =
            new Vision(drive::addVisionMeasurement, VisionConstants.Comp2025Mount.IO_CONSTANTS);
        // move to pos coral
        NamedCommands.registerCommand(
            "Coral to L4",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L4));

        NamedCommands.registerCommand(
            "Coral to L3",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L3));

        NamedCommands.registerCommand(
            "Coral to L2",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L2));

        NamedCommands.registerCommand(
            "Coral to L1",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L1));

        NamedCommands.registerCommand(
            "Coral to Ground",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_GROUND));

        NamedCommands.registerCommand(
            "Coral to Stow",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_STOW));

        // move to pos Algae
        NamedCommands.registerCommand(
            "Algae to L2 ",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_L2));

        NamedCommands.registerCommand(
            "Algae to L3",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_L3));

        NamedCommands.registerCommand(
            "Algae to Ground",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_GROUND));

        NamedCommands.registerCommand(
            "Algae to Processor",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_PROCESSOR));

        NamedCommands.registerCommand(
            "Algea to Stow",
            new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_STOW));

        // Collector
        NamedCommands.registerCommand(
            "Expel Coral", collector.expelCoralCommand(true, elevator::getEndEffectorSetpoint));

        NamedCommands.registerCommand("Expel Algae", collector.expelAlgaeCommand(true));

        NamedCommands.registerCommand(
            "Intake Coral", collector.intakeCoralCommand(true, elevator::getEndEffectorSetpoint));

        NamedCommands.registerCommand(
            "Intake Algae ", collector.intakeAlgaeCommand(true, elevator::getEndEffectorSetpoint));

        NamedCommands.registerCommand(
            "AutomatedLidarScoring right L4",
            new AutomatedLidarScoring(
                drive,
                collector,
                (DoubleSupplier) () -> 0.0,
                (DoubleSupplier) () -> 0.0,
                () -> EndEffectorSetpointConstants.CORAL_L4,
                true,
                .5,
                (BooleanSupplier) () -> true));

        NamedCommands.registerCommand(
            "AutomatedLidarScoring left L4",
            new AutomatedLidarScoring(
                drive,
                collector,
                (DoubleSupplier) () -> 0.0,
                (DoubleSupplier) () -> 0.0,
                () -> EndEffectorSetpointConstants.CORAL_L4,
                false,
                .5,
                (BooleanSupplier) () -> true));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    Joystick translationJoystick = new Joystick(0);

    Climber climber = new Climber();
    new JoystickButton(buttonJoystick, OIConstants.OI.IDs.Buttons.CLIMBER_GOTO_MAX)
        .whileTrue(climber.raiseClimber());
    new JoystickButton(buttonJoystick, OIConstants.OI.IDs.Buttons.CLIMBER_GOTO_MIN)
        .whileTrue(climber.lowerClimber());

    Funnel funnel = new Funnel();
    new JoystickButton(buttonJoystick, OIConstants.OI.IDs.Buttons.FUNNEL_GO_TO_MAX)
        .onTrue(new MoveFunnelToSetpoint(funnel, FunnelConstants.FUNNEL_POSITION_HIGH_CONVERTED));
    new JoystickButton(buttonJoystick, OIConstants.OI.IDs.Buttons.FUNNEL_GO_TO_MIN)
        .onTrue(new MoveFunnelToSetpoint(funnel, FunnelConstants.FUNNEL_POSITION_LOW_CONVERTED));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -translationJoystick.getY(),
            () -> -translationJoystick.getX(),
            () -> -rotationJoystick.getX()));

    Trigger algaeToggle =
        new JoystickButton(
            buttonJoystick, frc.robot.utils.constants.OIConstants.OI.IDs.Buttons.ALGAE_TOGGLE);
    Trigger coralToggle = algaeToggle.negate();

    JoystickButton endEffectorToL3 = new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_L3);
    JoystickButton endEffectorToL2 = new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_L2);
    JoystickButton endEffectorToGround =
        new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_GROUND);
    JoystickButton endEffectorToScoreLow =
        new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_SCORE_LOW);
    JoystickButton endEffectorStow = new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_STOW);
    JoystickButton endEffectorToScoreHigh =
        new JoystickButton(buttonJoystick, OI.IDs.Buttons.GOTO_SCORE_HIGH);
    // coral positions
    endEffectorToGround
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_GROUND));

    endEffectorToL2
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L2));

    endEffectorToScoreLow
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L1));

    endEffectorStow
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_STOW));

    endEffectorToL3
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L3));

    endEffectorToScoreHigh
        .and(coralToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.CORAL_L4));

    // algae
    endEffectorToGround
        .and(algaeToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_GROUND));

    endEffectorToScoreLow
        .and(algaeToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_PROCESSOR));

    endEffectorToL2
        .and(algaeToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_L2));

    endEffectorToL3
        .and(algaeToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_L3));

    endEffectorToScoreHigh
        .and(algaeToggle)
        .onTrue(new MoveEndEffector(elevator, wrist, EndEffectorSetpointConstants.ALGAE_NET));

    JoystickButton expelButton = new JoystickButton(buttonJoystick, OI.IDs.Buttons.EXPEL);
    JoystickButton intakeButton = new JoystickButton(buttonJoystick, OI.IDs.Buttons.INTAKE);

    intakeButton
        .and(coralToggle)
        .whileTrue(collector.intakeCoralCommand(true, elevator::getEndEffectorSetpoint));

    expelButton
        .and(coralToggle)
        .whileTrue(collector.expelCoralCommand(true, elevator::getEndEffectorSetpoint));

    intakeButton
        .and(algaeToggle)
        .whileTrue(collector.intakeAlgaeCommand(true, elevator::getEndEffectorSetpoint));

    expelButton.and(algaeToggle).whileTrue(collector.expelAlgaeCommand(true));

    double AUTO_JOYSTICK_SCALE = 0.25;
    new JoystickButton(translationJoystick, OI.IDs.Buttons.LIDAR_SCORE_LEFT)
        .whileTrue(
            new AutomatedLidarScoring(
                drive,
                collector,
                (DoubleSupplier) () -> translationJoystick.getX() * AUTO_JOYSTICK_SCALE,
                (DoubleSupplier) () -> translationJoystick.getY() * AUTO_JOYSTICK_SCALE,
                () -> EndEffectorSetpointConstants.CORAL_L4,
                false,
                .5,
                (BooleanSupplier) () -> true));

    new JoystickButton(translationJoystick, OI.IDs.Buttons.LIDAR_SCORE_RIGHT)
        .whileTrue(
            new AutomatedLidarScoring(
                drive,
                collector,
                (DoubleSupplier) () -> translationJoystick.getX() * AUTO_JOYSTICK_SCALE,
                (DoubleSupplier) () -> translationJoystick.getY() * AUTO_JOYSTICK_SCALE,
                () -> EndEffectorSetpointConstants.CORAL_L4,
                true,
                .5,
                (BooleanSupplier) () -> true));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset  to 0° when B button is pressed
    //     controller
    //         .b()
    //         .onTrue(
    //             Commands.runOnce(
    //                     () ->
    //                         drive.setPose(
    //                             new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                     drive)
    //                 .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
