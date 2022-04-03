// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autonomous.Turn;
import frc.robot.commands.drivetrain.AimAtBall;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.xboxControls;
import frc.robot.subsystems.vision.BallVision;
import frc.robot.subsystems.vision.TargetVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private XboxController _controller;
  public static DriveTrain _driveTrain = DriveTrain.getInstance();
  public static TargetVision _targetVision = TargetVision.getInstance();
  public static BallVision _ballVision = BallVision.getInstance();
  
  private final DriveTrain m_drivetrain = new DriveTrain();

  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  private final Turn m_turn = new Turn(
    m_drivetrain, () -> .5, () -> .5
  );


  public RobotContainer() {
    
    m_chooser.setDefaultOption("Turn", m_turn);

    SmartDashboard.putData(m_chooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(xboxControls.xboxController, 1).whenPressed(m_turn);
    
    _driveTrain.setDefaultCommand(new AimAtBall(_driveTrain, _ballVision, _controller));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
