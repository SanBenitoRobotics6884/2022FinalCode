// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick m_joystick = new Joystick(0);
  CANSparkMax m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_leftBack = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rightBack = new CANSparkMax(4, MotorType.kBrushless);
  MecanumDrive m_Drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_leftBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);
  }


  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_Drive.driveCartesian(-m_joystick.getRawAxis(1), m_joystick.getRawAxis(0), m_joystick.getRawAxis(4));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
