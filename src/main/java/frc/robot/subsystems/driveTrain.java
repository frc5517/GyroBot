// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import static frc.robot.Constants.kGains_visionCargo;
import static frc.robot.Constants.kGains_visionDrive;
import static frc.robot.Constants.kGains_visionTurn;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  PIDController m_speedPidController, m_turnPidController, m_cargoController;
	double visionDrivekP, visionDrivekI, visionDrivekD, visionTurnkP, visionTurnkI, visionTurnkD;

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

    m_speedPidController = new PIDController(kGains_visionDrive.kP, kGains_visionDrive.kI, kGains_visionDrive.kD);
			m_turnPidController = new PIDController(kGains_visionTurn.kP, kGains_visionTurn.kI, kGains_visionTurn.kD);
			m_cargoController = new PIDController(kGains_visionCargo.kP, kGains_visionCargo.kI, kGains_visionCargo.kD);

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
    if (XboxControls.xboxController.getRawButton(5)) {
      drive.setMaxOutput(.4);
    }
    else if (XboxControls.xboxController.getRawButton(6)) {
      drive.setMaxOutput(1);
    }
    else {
      drive.setMaxOutput(.7);
    }

    

    double forwardspeed = XboxControls.xboxController.getRawAxis(1);
    double turnspeed = XboxControls.xboxController.getRawAxis(4);

    drive.arcadeDrive(forwardspeed, turnspeed);

  }


  public void cargoAim(double yaw, double forward) {
		double forwardspeed = forward;
		double turnspeed = 0;

		if (yaw != 0) {
			turnspeed = -m_cargoController.calculate(yaw, 0);
		}else {
			turnspeed = 0;
		}

		NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Vision");
		visionTable.getEntry("forward drive speed").forceSetDouble(forwardspeed);
		visionTable.getEntry("Turn speed").forceSetDouble(turnspeed);
		visionTable.getEntry("Cargo Yaw").forceSetDouble(yaw);
		teleop_drive(forwardspeed, turnspeed);
	}

  private void teleop_drive(double forwardspeed, double turnspeed) {
    forwardspeed = XboxControls.xboxController.getRawAxis(1);
    turnspeed = XboxControls.xboxController.getRawAxis(4);
  }

  public void resetVisionPidController() {
		m_speedPidController.reset();
		m_turnPidController.reset();
	}

  }

