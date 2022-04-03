// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoCommands.DumpAndGo;
import frc.robot.AutoCommands.IntakeAndDump;
import frc.robot.AutoCommands.SimpleAuto;
import frc.robot.commands.CalibrateGyro;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ManualLift;
import frc.robot.commands.ResetPose;
import frc.robot.commands.UpdateWaypoint;
import frc.robot.subsystems.CargoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeedbackSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private PowerDistribution m_pdh = new PowerDistribution();

  private final Joystick m_joystick = new Joystick(0);
  private final XboxController m_controller = new XboxController(1);

  private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);
  private final CargoSubsystem m_cargo = new CargoSubsystem();
  private final LiftSubsystem m_lift = new LiftSubsystem();
  private final FeedbackSubsystem m_feedback = new FeedbackSubsystem(m_gyro, m_pdh, m_controller, m_cargo);

  private final Command m_resetPose = new ResetPose(m_drive);
  private final Command m_calibrateGyro = new CalibrateGyro(m_drive);
  private final Command m_updateWaypoint = new UpdateWaypoint(m_drive);
  
  private final Command m_simpleAuto = new SimpleAuto(m_drive, m_cargo);
  private final Command m_dumpAndGo = new DumpAndGo(m_drive, m_cargo);
  private final Command m_intakeDump = new IntakeAndDump(m_drive, m_cargo);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_autoChooser.setDefaultOption("None", null);
    m_autoChooser.addOption("Simple Auto", m_simpleAuto);
    m_autoChooser.addOption("Dump and Go", m_dumpAndGo);
    m_autoChooser.addOption("Intake and Dump", m_intakeDump);
    SmartDashboard.putData(m_autoChooser);
    
    m_gyro.calibrate();
    m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);

    configureButtonBindings();

 
    m_drive.setDefaultCommand(new DefaultDrive(m_drive,
      () -> m_controller.getLeftY(),
      () -> m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis(),
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
    new JoystickButton(m_controller, XboxController.Button.kLeftStick.value).
      whenPressed(new InstantCommand(() -> m_drive.setMaxSpeed(Constants.Drive.kTurboSpd) ))
      .whenReleased(new InstantCommand(() -> m_drive.setMaxSpeed(Constants.Drive.kFastSpd) ));

    /* Field centric removed for auto bandaid
    new JoystickButton(m_controller, XboxController.Button.kX.value).
      whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.EXPERIMENTAL) ));
    new JoystickButton(m_controller, XboxController.Button.kY.value).
      whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.EXPERIMENTALGYROASSIST) ));
    */

    new JoystickButton(m_joystick, 8).whenPressed(m_resetPose);
    new JoystickButton(m_joystick, 10).whenPressed(m_calibrateGyro);
    
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
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
