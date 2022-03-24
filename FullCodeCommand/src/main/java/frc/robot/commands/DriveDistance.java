// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveDistance extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;

  double m_distX, m_distY, m_rot;
  boolean waypointMode = false;

  public DriveDistance(DriveSubsystem subsystem, double distX, double distY, double rot) {
    m_distX = distX;
    m_distY = distY;
    m_rot = rot;

    m_drive = subsystem;
    addRequirements(m_drive);
  }

  public DriveDistance(DriveSubsystem subsystem) {
    waypointMode = true;

    m_drive = subsystem;
    addRequirements(m_drive);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (waypointMode) {
      Pose2d waypointPose = m_drive.getWaypointPose();
      m_drive.setPositionTarget(waypointPose.getX(), waypointPose.getY(), m_drive.getWaypointAngle());
    } else {
      m_drive.setPositionTarget(m_distY, m_distX, m_rot);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveTowardTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.atTargetPosition(m_distY, m_distX, m_rot);
  }
}
