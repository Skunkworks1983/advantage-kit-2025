// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.PIDControllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public interface SmartPIDBase {

  public default void putValueSmartDashboard(String PIDName, String name, double value) {
    SmartDashboard.putNumber("SmartPid/" + PIDName + "/" + name, value);
  }

  public default double getValueFromSmartDashboard(
      String PIDName, String name, double defaultValue) {
    return SmartDashboard.getNumber("SmartPid/" + PIDName + "/" + name, defaultValue);
  }
}
