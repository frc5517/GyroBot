// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.drivetrain.Rotate;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Turn extends ParallelDeadlineGroup {
  /** Creates a new Rotate. */
  public Turn(DriveTrain drivetrain, DoubleSupplier throttle, DoubleSupplier rotation) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(3));
    addCommands(
      new Rotate(drivetrain)
    );
  }
}

