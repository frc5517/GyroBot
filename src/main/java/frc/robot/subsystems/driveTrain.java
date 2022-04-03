// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static Spark leftRearMotor = new Spark(Constants.leftRearMotorPort);
  static Spark leftFrontMotor = new Spark(Constants.leftFrontMotorPort);
  static MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  static Spark rightRearMotor = new Spark(Constants.rightRearMotorPort);
  static Spark rightFrontMotor = new Spark(Constants.rightFrontMotorPort);
  static MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
  public DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


  public DriveTrain() {

    SmartDashboard.putData(gyro);

    gyro.calibrate();

  }

  public void setMaxOutput(double maxOutput) {
  }

  public void stop() {
    // Call DifferentialDrive's stopMotor method
    drive.stopMotor();
  }

  public void arcadeDrive(double throttle, double rotation) {
    // Call DifferentialDrive's arcadeDrive method
    drive.arcadeDrive(throttle, rotation, true);
  }
  
  @Override
  public void periodic() {

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    
    // Drive slower
    if (xboxControls.xboxController.getRawButton(5)) {
      drive.setMaxOutput(.4);
    }
    else if (xboxControls.xboxController.getRawButton(6)) {
      drive.setMaxOutput(1);
    }
    else {
      drive.setMaxOutput(.7);
    }

        double forwardSpeed;
        double rotationSpeed;

        forwardSpeed = -xboxControls.xboxController.getRawAxis(1);

        if (xboxControls.xboxController.getRawButton(2)) {
          // Vision-alignment mode
          // Query the latest result from PhotonVision
          var result = camera.getLatestResult();

          if (result.hasTargets()) {
              // Calculate angular turn power
              // -1.0 required to ensure positive PID controller effort _increases_ yaw
              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
          } else {
              // If we have no targets, stay still.
              rotationSpeed = 0;
          }
      } else {
          // Manual Driver Mode
          rotationSpeed = xboxControls.xboxController.getRawAxis(4);
      }

      // Use our forward/turn speeds to control the drivetrain
      drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}
