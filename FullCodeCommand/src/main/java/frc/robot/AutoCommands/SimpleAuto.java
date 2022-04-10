// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new DumpAndGo. */
  public SimpleAuto(DriveSubsystem driveSubsystem, CargoSubsystem cargoSubsystem) {
    addCommands(
      new InstantCommand(() -> cargoSubsystem.setIntakeStatus(true)),
      new DriveDistance(
        driveSubsystem,
        Constants.Auto.kSimpleDistX,
        Constants.Auto.kSimpleDistY,
        Constants.Auto.kSimpleAngle),
        new InstantCommand(() -> cargoSubsystem.setIntakeStatus(false))
    );
  }
}