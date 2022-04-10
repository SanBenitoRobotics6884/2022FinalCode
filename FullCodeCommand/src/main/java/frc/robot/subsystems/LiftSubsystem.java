// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  
  private CANSparkMax m_leftLiftMtr = new CANSparkMax(Constants.Lift.kLeftLiftID, MotorType.kBrushless);
  private CANSparkMax m_rightLiftMtr = new CANSparkMax(Constants.Lift.kRightLiftID, MotorType.kBrushless);
  private RelativeEncoder m_leftLiftEncoder;
  private RelativeEncoder m_rightLiftEncoder;

  private DigitalInput m_leftLimit = new DigitalInput(Constants.Lift.kLeftLimitPort);
  private DigitalInput m_rightLimit = new DigitalInput(Constants.Lift.kRightLimitPort);

  private double prevLiftSpeed = 0;
  private boolean isolateLeftArm = false;
  private boolean isolateRightArm = false;
  private boolean leftHasZeroed = false;
  private boolean rightHasZeroed = false;

  public LiftSubsystem() {
    m_leftLiftMtr.restoreFactoryDefaults();
    m_rightLiftMtr.restoreFactoryDefaults();

    m_leftLiftMtr.setInverted(false);
    m_rightLiftMtr.setInverted(true);

    m_leftLiftMtr.setIdleMode(IdleMode.kBrake);
    m_rightLiftMtr.setIdleMode(IdleMode.kBrake);

    m_rightLiftEncoder = m_rightLiftMtr.getEncoder();

    m_leftLiftEncoder = m_leftLiftMtr.getEncoder();
  }

  @Override
  public void periodic() {
    if (!m_leftLimit.get()) {
      m_leftLiftEncoder.setPosition(0);
      leftHasZeroed = true;
    }
    if (!m_rightLimit.get()) {
      m_rightLiftEncoder.setPosition(0);
      rightHasZeroed = true;
    }
    // This method will be called once per scheduler run
  }

  public void openLoopLift(double speed) {

    //speed = 0; // DELETE

    //Deadband
    if (Math.abs(speed) < 0.1) {
      speed = 0;
    }
    
    if (speed < 0) {

      if (!isolateRightArm && m_leftLimit.get()) {
        m_leftLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageLeft);
      } else {
        m_leftLiftMtr.setVoltage(0);
      }
      if (!isolateLeftArm && m_rightLimit.get()) {
        m_rightLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageRight);
      } else {
        m_rightLiftMtr.setVoltage(0);
      }

    } else if (leftHasZeroed && rightHasZeroed) { // Extending Lift
      if (!isolateRightArm && m_leftLiftEncoder.getPosition() < Constants.Lift.kMaxHeight) {
        m_leftLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageLeft);
      } else {
        m_leftLiftMtr.setVoltage(0);
      }
      if (!isolateLeftArm && m_rightLiftEncoder.getPosition() < Constants.Lift.kMaxHeight) {
        m_rightLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageRight);
      } else {
        m_rightLiftMtr.setVoltage(0);
      }

    } else {
      m_rightLiftMtr.setVoltage(0);
      m_leftLiftMtr.setVoltage(0);
    }

    // if (speed == 0 && m_rightLimit.get()) {
    //   m_rightLiftMtr.setVoltage(-1);
    // }
    // if (speed == 0 && m_leftLimit.get()) {
    //   m_leftLiftMtr.setVoltage(-1);
    // }

    prevLiftSpeed = speed;

  }
  public void setLeftArmStatus(boolean isIsolated) {
    isolateLeftArm = isIsolated;
  }

  public void setRightArmStatus(boolean isIsolated) {
    isolateRightArm = isIsolated;
  }

  public boolean getLeftArmStatus() {
    return isolateLeftArm;
  }
  public boolean getRightArmStatus() {
    return isolateRightArm;
  }

}
