package frc.robot.utils.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public class CurrentLimitConstants {
  public class CurrentLimits {

    // Measured in amps
    public static final double KRAKEN_CURRENT_LIMIT_VALUE = 90.0;
    public static final double KRAKEN_CURRENT_LIMIT_DRIVEBASE_DRIVE_VALUE = 75.0;
    public static final double MINI_KRAKEN_CURRENT_LIMIT_VALUE = 70.0;
    public static final int NEO_550_CURRENT_LIMIT_VALUE = 25; // not used

    public static final CurrentLimitsConfigs KRAKEN_CURRENT_LIMIT_CONFIG;
    public static final CurrentLimitsConfigs MINI_KRAKEN_CURRENT_LIMIT_CONFIG;
    public static final CurrentLimitsConfigs KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE;

    static {
      KRAKEN_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs();
      KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimit =
          KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimit = KRAKEN_CURRENT_LIMIT_VALUE;

      KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimitEnable =
          KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimitEnable = true;

      MINI_KRAKEN_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs();
      MINI_KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimit =
          MINI_KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimit = MINI_KRAKEN_CURRENT_LIMIT_VALUE;

      MINI_KRAKEN_CURRENT_LIMIT_CONFIG.StatorCurrentLimitEnable =
          MINI_KRAKEN_CURRENT_LIMIT_CONFIG.SupplyCurrentLimitEnable = true;

      KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE = new CurrentLimitsConfigs();
      KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE.StatorCurrentLimit =
          KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE.SupplyCurrentLimit =
              KRAKEN_CURRENT_LIMIT_DRIVEBASE_DRIVE_VALUE;

      KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE.StatorCurrentLimitEnable =
          KRAKEN_CURRENT_LIMIT_CONFIG_DRIVEBASE_DRIVE.SupplyCurrentLimitEnable = true;
    }
    ;
  }
}
