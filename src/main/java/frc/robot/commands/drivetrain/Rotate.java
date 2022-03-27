// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Rotate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // Create an object for the driveTrain
  private final DriveTrain m_driveTrain;

  // Create two DoubleSupplier objects for power being applied to motors

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveTrain The subsystem used by this command.
   */
  public Rotate(DriveTrain driveTrain) {
    // Use the driveTrain subsystem to gain access to its commands
    m_driveTrain = driveTrain;

    // Apply power to the motors
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use the Rotate method from the driveTrain subsystem

    double error = 180 - m_driveTrain.gyro.getAngle();

    m_driveTrain.drive.tankDrive(Constants.kTurnP * error, -Constants.kTurnP * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Use the stop method from the driveTrain subsystem
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
