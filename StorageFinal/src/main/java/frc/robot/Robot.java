// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  static final double uprMtrSpd = 0.7;
  static final double lwrMtrSpd = 0.3;
  Joystick m_joystick = new Joystick(0);
  WPI_TalonSRX m_upperMotor = new WPI_TalonSRX(5);
  WPI_TalonSRX m_lowerMotor = new WPI_TalonSRX(6);
  
 
  @Override
  public void robotInit() {
   m_lowerMotor.configFactoryDefault();
   m_upperMotor.configFactoryDefault();

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

    if (m_joystick.getRawButton(2)) {
      m_lowerMotor.set(lwrMtrSpd);
    } else {
      m_lowerMotor.set(0);
    }
    if (m_joystick.getTrigger()) {
      m_upperMotor.set(uprMtrSpd);
    } else {
      m_upperMotor.set(0);
    }
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
