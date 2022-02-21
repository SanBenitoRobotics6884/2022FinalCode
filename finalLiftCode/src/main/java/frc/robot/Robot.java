// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Date;
import java.sql.Time;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {
  Joystick m_joystick = new Joystick(0);

  CANSparkMax m_leftLift = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax m_rightLift = new CANSparkMax(8, MotorType.kBrushless);

  Servo m_leftAcuator = new Servo(0);
  Servo m_rightAcuator = new Servo(1);

  Timer time = new Timer();

  double targetTime = 0;
  double prevLiftSpeed = 0;

  @Override
  public void robotInit() {
    m_leftLift.restoreFactoryDefaults();
    m_rightLift.restoreFactoryDefaults();
    m_leftAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_rightAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    
  }


  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time", targetTime);
    SmartDashboard.putNumber("match time", Timer.getMatchTime());
  }


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
    double lift = m_joystick.getY();
    if (Math.abs(lift) < 0.1) {
      lift = 0;
    }

    if (lift >= 0) {
      m_leftAcuator.setSpeed(1);
      m_rightAcuator.setSpeed(1);

      m_leftLift.set(lift);
      m_rightLift.set(lift);
    } else {
      m_leftAcuator.setSpeed(-1);
      m_rightAcuator.setSpeed(-1);
      if (prevLiftSpeed >= 0) {
        targetTime = Timer.getMatchTime() + 1;
      }

      if (Timer.getMatchTime() >= targetTime) {
        m_leftLift.set(lift);
        m_rightLift.set(lift);
      } else {
        m_leftLift.set(0);
        m_rightLift.set(0);
      }

    }

    prevLiftSpeed = lift;

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
