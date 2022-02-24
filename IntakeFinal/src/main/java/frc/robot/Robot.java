// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private CANSparkMax m_intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  private Joystick m_joystick = new Joystick(0);
  private static final double kP = 0.0005;
  private static final double kI = 0;
  private static final double kD = 0.0000;
  private static final double kF = 0.0004;
  private static final double kMinOutput = -1;
  private static final double kMaxOutput = 1;
  private static final double kTargetRPM = 1000;
  private static final double kMotorGearing = 3;
  private static final double kOpenLoopSpeed = 0.5;
  private static final double kMaxVoltage = 12;
  private SparkMaxPIDController m_Pid;
  private RelativeEncoder m_encoder;
  private static final boolean isPid = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_intakeMotor.restoreFactoryDefaults();
    m_encoder = m_intakeMotor.getEncoder();
    m_encoder.setVelocityConversionFactor(1/kMotorGearing);
    m_Pid = m_intakeMotor.getPIDController();
    m_Pid.setP(kP);
    m_Pid.setI(kI);
    m_Pid.setD(kD);
    m_Pid.setIZone(0);
    m_Pid.setFF(kF);
    m_Pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("RPM", m_encoder.getVelocity());
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
      if (m_joystick.getTrigger()) {
        m_Pid.setReference(kTargetRPM, CANSparkMax.ControlType.kVelocity);
      } else {
        m_intakeMotor.setVoltage(0);
      }
    } else {
      if (m_joystick.getRawButton(2)) {
        m_intakeMotor.setVoltage(kOpenLoopSpeed * kMaxVoltage);
      } else {
        m_intakeMotor.setVoltage(0);
      }
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
