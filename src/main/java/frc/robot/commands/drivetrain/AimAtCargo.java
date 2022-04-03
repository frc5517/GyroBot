// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.XboxControls;

public class AimAtCargo extends CommandBase {
  /** Creates a new AimAtCargo. */

  Vision m_vision;
  DriveTrain m_driveTrain;
  XboxControls m_xboxController;

  public AimAtCargo(Vision vision, DriveTrain driveTrain, XboxControls xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_vision = vision;
    m_driveTrain = driveTrain;
    m_xboxController = xboxController;

    addRequirements(vision, driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetVisionPidController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forward = XboxControls.xboxController.getRawAxis(1);
    m_driveTrain.cargoAim(m_vision.getCargoTargetYaw(), forward);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
