package frc.robot.utils;

public class EndEffectorSetpoints {

  public static final double WRIST_STOW_POSITION_CORAL = 0.0;
  public static final double WRIST_STOW_POSITION_ALGAE = 0.0;
  public static final double WRIST_PROSSESSOR_STOW_POSITION = 0.1441;

  public static final EndEffectorSetpointConstants ALGAE_GROUND =
      new EndEffectorSetpointConstants(3.0, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorSetpointConstants ALGAE_STOW =
      new EndEffectorSetpointConstants(
          0.0, WRIST_PROSSESSOR_STOW_POSITION, WRIST_PROSSESSOR_STOW_POSITION);
  public static final EndEffectorSetpointConstants ALGAE_PROCESSOR =
      new EndEffectorSetpointConstants(8.8, 0.3941, WRIST_PROSSESSOR_STOW_POSITION);
  public static final EndEffectorSetpointConstants ALGAE_L2 =
      new EndEffectorSetpointConstants(19.018 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorSetpointConstants ALGAE_L3 =
      new EndEffectorSetpointConstants(29.77 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
  public static final EndEffectorSetpointConstants ALGAE_NET =
      new EndEffectorSetpointConstants(34.29, 0.1441, 0.1441);

  public static final EndEffectorSetpointConstants CORAL_GROUND =
      new EndEffectorSetpointConstants(0.0, 0.3941, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorSetpointConstants CORAL_STOW =
      new EndEffectorSetpointConstants(0.0, WRIST_STOW_POSITION_CORAL, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorSetpointConstants CORAL_L1 =
      new EndEffectorSetpointConstants(12.0, 0.0641, WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorSetpointConstants CORAL_L2 =
      new EndEffectorSetpointConstants(
          12.756 + 0.5 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorSetpointConstants CORAL_L3 =
      new EndEffectorSetpointConstants(
          23.914 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
  public static final EndEffectorSetpointConstants CORAL_L4 =
      new EndEffectorSetpointConstants(39.0, 0.0, WRIST_STOW_POSITION_CORAL);
}
;
