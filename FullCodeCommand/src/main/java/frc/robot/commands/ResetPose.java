// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetPose extends CommandBase {

  private DriveSubsystem m_drive;

  public ResetPose(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;
  }

  @Override
  public void initialize() {
    m_drive.calibrateGyro();
    m_drive.resetPose();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }
}
