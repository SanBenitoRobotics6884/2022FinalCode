// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Date;
import java.sql.Time;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final double kMinOutput = -1;
  private static final double kMaxOutput = 1;
  private static final boolean isPid = false;
  private SparkMaxPIDController m_leftPid;
  private SparkMaxPIDController m_rightPid;
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

    m_rightEncoder = m_rightLift.getEncoder();
    m_rightPid = m_rightLift.getPIDController();
    m_rightPid.setP(kP);
    m_rightPid.setI(kI);
    m_rightPid.setD(kD);
    m_rightPid.setIZone(0);
    m_rightPid.setFF(kF);
    m_rightPid.setOutputRange(kMinOutput, kMaxOutput);

    m_leftEncoder = m_leftLift.getEncoder();
    m_leftPid = m_leftLift.getPIDController();
    m_leftPid.setP(kP);
    m_leftPid.setI(kI);
    m_leftPid.setD(kD);
    m_leftPid.setIZone(0);
    m_leftPid.setFF(kF);
    m_leftPid.setOutputRange(kMinOutput, kMaxOutput);

    
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
    if (isPid) {

      

    } else {
      openLoopLift();
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

  public void openLoopLift() {
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

}
