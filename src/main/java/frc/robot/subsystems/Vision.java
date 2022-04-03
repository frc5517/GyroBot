// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  int m_pipeline = 0;

  PhotonCamera m_camera = new PhotonCamera("USB Camera");

  NetworkTableEntry m_tx, m_ty, m_ta, m_cargoYaw, 
  m_cargoHasTargets, m_bestTarget, targetPitch, targetYaw, 
  targetRange, targetCount, cargoTarget, hubTarget;

  PhotonTrackedTarget m_target;

  double pitch, yaw, area;

  public Vision() {

    m_camera.setPipelineIndex(m_pipeline);

    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.2.0");

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive");
    cargoTarget = table.getEntry("BallTarget");

    NetworkTable cargoTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("lifecam");
    m_cargoYaw = cargoTable.getEntry("targetYaw");
    m_cargoHasTargets = cargoTable.getEntry("hasTarget");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    getCargoTargetStatus();

  }

  private boolean getCargoTargetStatus() {
    var result = m_camera.getLatestResult();
     boolean hasTargets = result.hasTargets();
     cargoTarget.setBoolean(hasTargets);
    return hasTargets;
  }

  public double getCargoTargetYaw() {
    double Yaw = m_cargoYaw.getDouble(0);
    return Yaw;
  }

  public void setTeamPipeline() {
    int pipelineIndex;
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      pipelineIndex = 1;
      m_camera.setPipelineIndex(pipelineIndex);
    }else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      pipelineIndex = 0;
      m_camera.setPipelineIndex(pipelineIndex);
    }
  }

}
