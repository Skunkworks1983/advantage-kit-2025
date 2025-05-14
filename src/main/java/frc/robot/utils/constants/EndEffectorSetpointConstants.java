package frc.robot.utils.constants;

public class EndEffectorSetpointConstants {
  public class EndEffectorSetpoints {

    public static final double WRIST_STOW_POSITION_CORAL = 0.0;
    public static final double WRIST_STOW_POSITION_ALGAE = 0.0;
    public static final double WRIST_PROSSESSOR_STOW_POSITION = 0.1441;

    public static final EffectorToSetpointConstants ALGAE_GROUND =
        new EffectorToSetpointConstants(3.0, 0.3941, WRIST_STOW_POSITION_ALGAE);
    public static final EffectorToSetpointConstants ALGAE_STOW =
        new EffectorToSetpointConstants(
            0.0, WRIST_PROSSESSOR_STOW_POSITION, WRIST_PROSSESSOR_STOW_POSITION);
    public static final EffectorToSetpointConstants ALGAE_PROCESSOR =
        new EffectorToSetpointConstants(8.8, 0.3941, WRIST_PROSSESSOR_STOW_POSITION);
    public static final EffectorToSetpointConstants ALGAE_L2 =
        new EffectorToSetpointConstants(19.018 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
    public static final EffectorToSetpointConstants ALGAE_L3 =
        new EffectorToSetpointConstants(29.77 + 5, 0.3941, WRIST_STOW_POSITION_ALGAE);
    public static final EffectorToSetpointConstants ALGAE_NET =
        new EffectorToSetpointConstants(34.29, 0.1441, 0.1441);

    public static final EffectorToSetpointConstants CORAL_GROUND =
        new EffectorToSetpointConstants(0.0, 0.3941, WRIST_STOW_POSITION_CORAL);
    public static final EffectorToSetpointConstants CORAL_STOW =
        new EffectorToSetpointConstants(0.0, WRIST_STOW_POSITION_CORAL, WRIST_STOW_POSITION_CORAL);
    public static final EffectorToSetpointConstants CORAL_L1 =
        new EffectorToSetpointConstants(12.0, 0.0641, WRIST_STOW_POSITION_CORAL);
    public static final EffectorToSetpointConstants CORAL_L2 =
        new EffectorToSetpointConstants(
            12.756 + 0.5 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
    public static final EffectorToSetpointConstants CORAL_L3 =
        new EffectorToSetpointConstants(
            23.914 + 0.75, (WRIST_PROSSESSOR_STOW_POSITION / 2), WRIST_STOW_POSITION_CORAL);
    public static final EffectorToSetpointConstants CORAL_L4 =
        new EffectorToSetpointConstants(39.0, 0.0, WRIST_STOW_POSITION_CORAL);
  }
  ;
}
