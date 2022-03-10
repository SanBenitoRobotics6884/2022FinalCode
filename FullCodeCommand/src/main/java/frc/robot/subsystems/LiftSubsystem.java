// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  
  private CANSparkMax m_leftLiftMtr = new CANSparkMax(Constants.Lift.kLeftLiftID, MotorType.kBrushless);
  private CANSparkMax m_rightLiftMtr = new CANSparkMax(Constants.Lift.kRightLiftID, MotorType.kBrushless);
  private SparkMaxPIDController m_leftLiftPid;
  private SparkMaxPIDController m_rightLiftPid;
  private RelativeEncoder m_leftLiftEncoder;
  private RelativeEncoder m_rightLiftEncoder;

  private Servo m_leftAcuator = new Servo(Constants.Lift.kLeftServoPort);
  private Servo m_rightAcuator = new Servo(Constants.Lift.kRightServoPort);

  private DigitalInput m_leftLimit = new DigitalInput(Constants.Lift.kLeftLimitPort);
  private DigitalInput m_rightLimit = new DigitalInput(Constants.Lift.kRightLimitPort);

  private double targetLiftTime = 0;
  private double prevLiftSpeed = 0;
  private boolean prevRatchet = true;
  private int closedLoopMode = 0; // 1: High, 0: Low
  private boolean isolateLeftArm = false;
  private boolean isolateRightArm = false;

  public LiftSubsystem() {
    m_leftLiftMtr.restoreFactoryDefaults();
    m_rightLiftMtr.restoreFactoryDefaults();

    m_leftLiftMtr.setInverted(true);
    m_rightLiftMtr.setInverted(false);

    m_leftAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_rightAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    m_rightLiftEncoder = m_rightLiftMtr.getEncoder();
    m_rightLiftPid = m_rightLiftMtr.getPIDController();
    m_rightLiftPid.setP(Constants.Lift.LiftPID.kPlift);
    m_rightLiftPid.setI(Constants.Lift.LiftPID.kIlift);
    m_rightLiftPid.setD(Constants.Lift.LiftPID.kDlift);
    m_rightLiftPid.setIZone(0);
    m_rightLiftPid.setFF(Constants.Lift.LiftPID.kFUnloadedLift);
    m_rightLiftPid.setOutputRange(Constants.Lift.LiftPID.kMinOutputLift, Constants.Lift.LiftPID.kMaxOutputLift);

    m_rightLiftPid.setSmartMotionMaxVelocity(Constants.Lift.LiftPID.kMaxVelLift, Constants.Lift.kSmartMotionSlot);
    m_rightLiftPid.setSmartMotionMaxAccel(Constants.Lift.LiftPID.kMaxAccLift, Constants.Lift.kSmartMotionSlot);
    m_rightLiftPid.setSmartMotionAllowedClosedLoopError(Constants.Lift.LiftPID.kAllowedErrLift, Constants.Lift.kSmartMotionSlot);

    m_leftLiftEncoder = m_leftLiftMtr.getEncoder();
    m_leftLiftPid = m_leftLiftMtr.getPIDController();
    m_leftLiftPid.setP(Constants.Lift.LiftPID.kPlift);
    m_leftLiftPid.setI(Constants.Lift.LiftPID.kIlift);
    m_leftLiftPid.setD(Constants.Lift.LiftPID.kDlift);
    m_leftLiftPid.setIZone(0);
    m_leftLiftPid.setFF(Constants.Lift.LiftPID.kFUnloadedLift);
    m_leftLiftPid.setOutputRange(Constants.Lift.LiftPID.kMinOutputLift, Constants.Lift.LiftPID.kMaxOutputLift);

    m_leftLiftPid.setSmartMotionMaxVelocity(Constants.Lift.LiftPID.kMaxVelLift, Constants.Lift.kSmartMotionSlot);
    m_leftLiftPid.setSmartMotionMaxAccel(Constants.Lift.LiftPID.kMaxAccLift, Constants.Lift.kSmartMotionSlot);
    m_leftLiftPid.setSmartMotionAllowedClosedLoopError(Constants.Lift.LiftPID.kAllowedErrLift, Constants.Lift.kSmartMotionSlot);

    //Deploy ratchets at match start
    m_leftAcuator.setSpeed(Constants.Lift.kRatchetDeploy);
    m_rightAcuator.setSpeed(Constants.Lift.kRatchetDeploy);
  }

  @Override
  public void periodic() {
    if (!m_leftLimit.get()) {
      m_leftLiftEncoder.setPosition(0);
    }
    if (!m_rightLimit.get()) {
      m_rightLiftEncoder.setPosition(0);
    }
    // This method will be called once per scheduler run
  }

  public void openLoopLift(double speed) {
    //Deadband
    if (Math.abs(speed) < 0.1) {
      speed = 0;
    }

    SmartDashboard.putNumber("Speed", speed * Constants.Lift.kMaxVoltageLeft);

    if (speed <= 0) { //Retracting Lift
      m_leftAcuator.setSpeed(Constants.Lift.kRatchetDeploy);
      m_rightAcuator.setSpeed(Constants.Lift.kRatchetDeploy);

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

    } else { //Raising Lift
      //Ratchets must be fully retracted to raise lift arms
      m_leftAcuator.setSpeed(Constants.Lift.kRatchetRetract);
      m_rightAcuator.setSpeed(Constants.Lift.kRatchetRetract);

      //Allow arms to move after 1 second has passed
      if (prevLiftSpeed <= 0) {
        targetLiftTime = Timer.getFPGATimestamp() + Constants.Lift.kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetLiftTime) {
        if (!isolateRightArm) {
          m_leftLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageLeft);
        } else {
          m_leftLiftMtr.setVoltage(0);
        }
        if (!isolateLeftArm) {
          m_rightLiftMtr.setVoltage(speed * Constants.Lift.kMaxVoltageRight);
        } else {
          m_rightLiftMtr.setVoltage(0);
        }
      } else {
        m_leftLiftMtr.setVoltage(0);
        m_rightLiftMtr.setVoltage(0);
      }

    }

    prevLiftSpeed = speed;

  }
  public void closedLoopLift() {

    if (closedLoopMode == 0) { //Retracting
      //If dynamic kF enabled, use higher FF gain to counteract
      //weight of robot while lifting
      if (prevRatchet == false && Constants.Lift.isDynamicKF) {
        m_leftLiftPid.setFF(Constants.Lift.LiftPID.kFLoadedLift);
        m_rightLiftPid.setFF(Constants.Lift.LiftPID.kFLoadedLift);
      }

      //Set target position to retract and deploy ratchet
      m_leftLiftPid.setReference(Constants.Lift.kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_rightLiftPid.setReference(Constants.Lift.kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_leftAcuator.setSpeed(Constants.Lift.kRatchetDeploy);
      m_rightAcuator.setSpeed(Constants.Lift.kRatchetDeploy);
      prevRatchet = true;
    }


    if (closedLoopMode == 1) { //Extending
      //Use default FF gain when lifting (no weight to account for)
      if (prevRatchet == true && Constants.Lift.isDynamicKF) {
        m_leftLiftPid.setFF(Constants.Lift.LiftPID.kFUnloadedLift);
        m_rightLiftPid.setFF(Constants.Lift.LiftPID.kFUnloadedLift);
      }

      m_leftAcuator.setSpeed(Constants.Lift.kRatchetRetract);
      m_rightAcuator.setSpeed(Constants.Lift.kRatchetRetract);

      //Delay 1 second so ratchets can fully deploy
      if (prevRatchet == true) {
        targetLiftTime = Timer.getFPGATimestamp() + Constants.Lift.kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetLiftTime) {
        m_leftLiftPid.setReference(Constants.Lift.kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        m_rightLiftPid.setReference(Constants.Lift.kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("Run");
      } else {
        m_leftLiftMtr.setVoltage(0);
        m_rightLiftMtr.setVoltage(0);
      }
      prevRatchet = false;
    }
  }
  public void setLeftArmStatus(boolean isRunning) {
    isolateLeftArm = isRunning;
  }

  public void setRightArmStatus(boolean isRunning) {
    isolateRightArm = isRunning;
  }

  public void setClosedLoopMode(int mode) {
    closedLoopMode = mode;
  }

  public boolean getLeftArmStatus() {
    return isolateLeftArm;
  }
  public boolean getRightArmStatus() {
    return isolateRightArm;
  }

}
