// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ManualLift;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final CargoSubsystem m_cargo = new CargoSubsystem();
  private final LiftSubsystem m_lift = new LiftSubsystem();
  private final Joystick m_joystick = new Joystick(0);
  private final XboxController m_controller = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new DefaultDrive(m_drive,
      () -> m_controller.getLeftY(),
      () -> m_controller.getLeftX(),
      () -> m_controller.getRightX())
    );

    m_lift.setDefaultCommand(new ManualLift(m_lift, () -> m_joystick.getY() ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value).
      whenPressed(new InstantCommand(() -> m_drive.setMaxSpeed(Constants.Drive.kSlowSpd) ));
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value).
      whenPressed(new InstantCommand(() -> m_drive.setMaxSpeed(Constants.Drive.kFastSpd) ));
    
    new JoystickButton(m_joystick, 2)
      .whenPressed(new InstantCommand(() -> m_cargo.setIntakeStatus(true) ))
      .whenReleased(new InstantCommand(() -> m_cargo.setIntakeStatus(false) ));

    new JoystickButton(m_joystick, 1)
      .whenPressed(new InstantCommand(() -> m_cargo.setLaunchStatus(true) ))
      .whenReleased(new InstantCommand(() -> m_cargo.setLaunchStatus(false) ));

    new JoystickButton(m_joystick, 7)
      .whenPressed(new InstantCommand(() -> m_cargo.setEvacStatus(true) ))
      .whenReleased(new InstantCommand(() -> m_cargo.setEvacStatus(false) ));

    new JoystickButton(m_joystick, 3)
      .whenPressed(new InstantCommand(() -> m_lift.setLeftArmStatus(true) ))
      .whenReleased(new InstantCommand(() -> m_lift.setLeftArmStatus(false) ));

    new JoystickButton(m_joystick, 4)
      .whenPressed(new InstantCommand(() -> m_lift.setRightArmStatus(true) ))
      .whenReleased(new InstantCommand(() -> m_lift.setRightArmStatus(false) ));

    new JoystickButton(m_joystick, 12)
      .whenPressed(new InstantCommand(() -> m_lift.disengageRatchets() ));
  }

}