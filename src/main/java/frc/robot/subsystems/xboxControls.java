// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class xboxControls extends SubsystemBase {
  /** Creates a new XboxController. */

  public static XboxController xboxController = new XboxController(OIConstants.xboxControllerPort);

  public xboxControls() {}

  @Override
  public void periodic() {}

}
