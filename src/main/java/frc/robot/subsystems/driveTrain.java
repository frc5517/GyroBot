// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(Constants.leftRearMotorPort);
  static WPI_VictorSPX leftFrontMotor = new WPI_VictorSPX(Constants.leftFrontMotorPort);
  static MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  static WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(Constants.rightRearMotorPort);
  static WPI_VictorSPX rightFrontMotor = new WPI_VictorSPX(Constants.rightFrontMotorPort);
  static MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
  public DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public DriveTrain() {
    leftRearMotor.setNeutralMode(NeutralMode.Coast);
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightRearMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);

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

    leftMotors.setInverted(false);
    rightMotors.setInverted(true);
    
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

        forwardSpeed = xboxControls.xboxController.getRawAxis(1);
        rotationSpeed = -xboxControls.xboxController.getRawAxis(4);

        // Use our forward/turn speeds to control the drivetrain
        drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
