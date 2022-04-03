// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.vision.BallVision;

public class AimAtBall extends CommandBase {
  /** Creates a new AimAtBall. */

  private DriveTrain _driveTrain;
  private XboxController _controller;
  private BallVision _ballVision;
  private double _targetYaw;
  private boolean _hasTarget;

  public static final double DEADZONE = 0.12;

  public AimAtBall(DriveTrain driveTrain, BallVision ballVision, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    this._driveTrain = driveTrain;
    this._ballVision = ballVision;
    this._controller = controller;

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this._ballVision.hasTargets()) {
      this._hasTarget = true;
      this._targetYaw = this._ballVision.getYawVal();
      this._driveTrain.visionTurn(() -> 0, this._hasTarget, DEADZONE, this._targetYaw);
    } else {
      this._hasTarget = false;

      if (this._controller.getRightTriggerAxis() > 0) {
        this._driveTrain.visionTurn(() -> this._controller.getRightTriggerAxis(), this._hasTarget, DEADZONE,
            this._targetYaw);
      } else if (this._controller.getLeftTriggerAxis() > 0) {
        this._driveTrain.visionTurn(() -> -(this._controller.getLeftTriggerAxis()), this._hasTarget, DEADZONE,
            this._targetYaw);
      } else {
        this._driveTrain.stopDriveTrain();
      }
    }

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
