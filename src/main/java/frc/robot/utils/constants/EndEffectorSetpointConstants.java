package frc.robot.utils.constants;

public class EndEffectorSetpointConstants {

  public static final double WRIST_STOW_POSITION_CORAL = 0.0;
  public static final double WRIST_STOW_POSITION_ALGAE = 0.1441;
  public static final double WRIST_PROSSESSOR_STOW_POSITION = 0.1441;

  public static final EndEffectorToSetpointConstants ALGAE_GROUND =
      new EndEffectorToSetpointConstants(3.0, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorToSetpointConstants ALGAE_STOW =
      new EndEffectorToSetpointConstants(
          0.0, WRIST_PROSSESSOR_STOW_POSITION, WRIST_PROSSESSOR_STOW_POSITION);
  public static final EndEffectorToSetpointConstants ALGAE_PROCESSOR =
      new EndEffectorToSetpointConstants(8.8, 0.3941, WRIST_PROSSESSOR_STOW_POSITION);
  public static final EndEffectorToSetpointConstants ALGAE_L2 =
      new EndEffectorToSetpointConstants(19.018 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorToSetpointConstants ALGAE_L3 =
      new EndEffectorToSetpointConstants(29.77 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorToSetpointConstants ALGAE_NET =
      new EndEffectorToSetpointConstants(34.29, 0.1441, 0.1441);

  public static final EndEffectorToSetpointConstants CORAL_GROUND =
      new EndEffectorToSetpointConstants(0.4, 0.3941, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorToSetpointConstants CORAL_STOW =
      new EndEffectorToSetpointConstants(0.4, WRIST_STOW_POSITION_CORAL, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorToSetpointConstants CORAL_L1 =
      new EndEffectorToSetpointConstants(13.75, 0.3941, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorToSetpointConstants CORAL_L2 =
      new EndEffectorToSetpointConstants(
          12.756 + 0.5 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorToSetpointConstants CORAL_L3 =
      new EndEffectorToSetpointConstants(
          23.914 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorToSetpointConstants CORAL_L4 =
      new EndEffectorToSetpointConstants(39.9, 0.1, WRIST_STOW_POSITION_CORAL);
}
