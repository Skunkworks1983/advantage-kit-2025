// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.constants;

/** Add your docs here. */
public class OIconstants {
  public class OI {

    public static final double AXIS_DEADBAND = .08;

    // (x or y joystick axis input after deadband ^ AXIS_INPUT_EXPONENT)
    // * MAX_INSTRUCTED_METERS_PER_SECOND = instructed meters per second
    // Ensure that this AXIS_INPUT_EXPONENT does not result in a result
    // that is always positive.
    public static final double AXIS_INPUT_EXPONENT = 3.0;

    public class IDs {
      public class Joysticks {
        public static final int ROTATION_JOYSTICK_ID = 1;
        public static final int TRANSLATION_JOYSTICK_ID = 0;
        public static final int BUTTON_STICK_ID = 2;
      }

      // TODO: add button IDs
      public class Buttons {

        // Switch being off corresponds to coral
        // Switch being on corresponds to algae
        public static final int ALGAE_TOGGLE = 8;

        // The following buttons depend on ALGAE_TOGGLE
        public static final int GOTO_SCORE_LOW = 15; // either L1 or proccesor on ALGAE_TOGGLE
        public static final int GOTO_L2 = 13; // either place coral L2 or remove algae L2
        public static final int GOTO_L3 = 12; // either place coral L3 or remove algae L4
        public static final int GOTO_SCORE_HIGH = 11; // either L4 or net depending on ALGAE_TOGGLE
        public static final int GOTO_STOW = 23;
        // f these buttons may change depending on algae or coral mode.
        // Will also change for different positions (e.g. net)
        public static final int INTAKE = 17;
        public static final int EXPEL = 24;

        public static final int CLIMBER_GOTO_MAX = 0;
        public static final int CLIMBER_GOTO_MIN = 0;

        public static final int TARGET_REEF_BUTTON = 3;
        public static final int TARGET_CORAL_STATION_BUTTON = 2;
        public static final int TARGET_CORAL_CYCLE_NO_ODOMETRY_BUTTON = 1;

        public static final int RAISE_FUNNEL_TOGGLE = 0;

        public static final int FUNNEL_GO_TO_MAX = 0;
        public static final int FUNNEL_GO_TO_MIN = 0;
      }
    }
  }
}
