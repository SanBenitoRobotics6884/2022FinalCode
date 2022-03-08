// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {
  
  //Constant for motion magic
  private static final double kPlift = 0.00015;
  private static final double kIlift = 0;
  private static final double kDlift = 0;
  private static final double kFUnloadedLift = 0;
  private static final double kFLoadedLift = 0.0004;
  private static final double kMinOutputLift = -1;
  private static final double kMaxOutputLift = 1;
  private static final double maxVelLift = 2000; // rpm
  private static final double maxAccLift = 1500;
  private static final double allowedErrLift = 0;

  private static final boolean isPid = false; //Run in PID mode instead of direct control (open loop)
  private static final boolean isDynamicKF = false; //EXPERIMENTAL. USE WITH CAUTION
  private static final double kLowSetpoint = 0; //Units: motor rotations
  private static final double kHighSetpoint = 20; //Units: motor rotations
  private static final double kMaxVoltageLeft = -4; //CHANGE BACK TO 4
  private static final double kMaxVoltageRight = -4.5; //CHANGE BACK TO 4
  private static final double kRatchetDeploy = 1;
  private static final double kRatchetRetract = -0.35;
  private static final double kRatchetDelay = 3;

  private static final int smartMotionSlot = 0;
  private static final int kLeftLiftID = 8;
  private static final int kRightLiftID = 9;
  private static final int kJoystickPort = 0;
  private static final int kLeftServoPort = 0;
  private static final int kRightServoPort = 1;

  private CANSparkMax m_leftLiftMtr = new CANSparkMax(kLeftLiftID, MotorType.kBrushless);
  private CANSparkMax m_rightLiftMtr = new CANSparkMax(kRightLiftID, MotorType.kBrushless);
  private SparkMaxPIDController m_leftLiftPid;
  private SparkMaxPIDController m_rightLiftPid;
  private RelativeEncoder m_leftLiftEncoder;
  private RelativeEncoder m_rightLiftEncoder;

  private Joystick m_joystick = new Joystick(kJoystickPort);

  private Servo m_leftAcuator = new Servo(kLeftServoPort);
  private Servo m_rightAcuator = new Servo(kRightServoPort);

  DigitalInput m_leftLimit = new DigitalInput(8);
  DigitalInput m_rightLimit = new DigitalInput(9);

  private double targetLiftTime = 0;
  private double prevLiftSpeed = 0;
  private boolean prevRatchet = true;
  private int closedLoopMode = 0; // 1: High, 0: Low

  @Override
  public void robotInit() {
    m_leftLiftMtr.restoreFactoryDefaults();
    m_rightLiftMtr.restoreFactoryDefaults();

    m_leftLiftMtr.setInverted(true);
    m_rightLiftMtr.setInverted(false);

    m_leftAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_rightAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    m_rightLiftEncoder = m_rightLiftMtr.getEncoder();
    m_rightLiftPid = m_rightLiftMtr.getPIDController();
    m_rightLiftPid.setP(kPlift);
    m_rightLiftPid.setI(kIlift);
    m_rightLiftPid.setD(kDlift);
    m_rightLiftPid.setIZone(0);
    m_rightLiftPid.setFF(kFUnloadedLift);
    m_rightLiftPid.setOutputRange(kMinOutputLift, kMaxOutputLift);

    m_rightLiftPid.setSmartMotionMaxVelocity(maxVelLift, smartMotionSlot);
    m_rightLiftPid.setSmartMotionMaxAccel(maxAccLift, smartMotionSlot);
    m_rightLiftPid.setSmartMotionAllowedClosedLoopError(allowedErrLift, smartMotionSlot);

    m_leftLiftEncoder = m_leftLiftMtr.getEncoder();
    m_leftLiftPid = m_leftLiftMtr.getPIDController();
    m_leftLiftPid.setP(kPlift);
    m_leftLiftPid.setI(kIlift);
    m_leftLiftPid.setD(kDlift);
    m_leftLiftPid.setIZone(0);
    m_leftLiftPid.setFF(kFUnloadedLift);
    m_leftLiftPid.setOutputRange(kMinOutputLift, kMaxOutputLift);

    m_leftLiftPid.setSmartMotionMaxVelocity(maxVelLift, smartMotionSlot);
    m_leftLiftPid.setSmartMotionMaxAccel(maxAccLift, smartMotionSlot);
    m_leftLiftPid.setSmartMotionAllowedClosedLoopError(allowedErrLift, smartMotionSlot);

    //Deploy ratchets at match start
    m_leftAcuator.setSpeed(kRatchetDeploy);
    m_rightAcuator.setSpeed(kRatchetDeploy);
  }


  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Position", m_leftLiftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightLiftEncoder.getPosition());
    SmartDashboard.putBoolean("Right LS", m_rightLimit.get());
    SmartDashboard.putBoolean("Left LS", m_leftLimit.get());

    if (!m_leftLimit.get()) {
      m_leftLiftEncoder.setPosition(0);
    }
    if (!m_rightLimit.get()) {
      m_rightLiftEncoder.setPosition(0);
    }
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
        closedLoopLift();
      } else {
        m_leftLiftMtr.setVoltage(0);
        m_rightLiftMtr.setVoltage(0);
      }
    } else {
      openLoopLift(m_joystick.getY());
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
  public void testPeriodic() {
    if (m_joystick.getRawButton(3)) {
      m_leftAcuator.setSpeed(m_joystick.getY());
      SmartDashboard.putNumber("Left Retract", m_joystick.getY());
    } else if (m_joystick.getRawButton(4)) {
      m_rightAcuator.setSpeed(m_joystick.getY());
      SmartDashboard.putNumber("Right Retract", m_joystick.getY());
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void openLoopLift(double speed) {
    //Deadband
    if (Math.abs(speed) < 0.1) {
      speed = 0;
    }

    SmartDashboard.putNumber("Speed", speed * kMaxVoltageLeft);

    if (speed <= 0) { //Retracting Lift
      m_leftAcuator.setSpeed(kRatchetDeploy);
      m_rightAcuator.setSpeed(kRatchetDeploy);

      if (!m_joystick.getRawButton(4) && m_leftLimit.get()) {
        m_leftLiftMtr.setVoltage(speed * kMaxVoltageLeft);
      } else {
        m_leftLiftMtr.setVoltage(0);
      }
      if (!m_joystick.getRawButton(3) && m_rightLimit.get()) {
        m_rightLiftMtr.setVoltage(speed * kMaxVoltageRight);
      } else {
        m_rightLiftMtr.setVoltage(0);
      }

    } else { //Raising Lift
      //Ratchets must be fully retracted to raise lift arms
      m_leftAcuator.setSpeed(kRatchetRetract);
      m_rightAcuator.setSpeed(kRatchetRetract);

      //Allow arms to move after 1 second has passed
      if (prevLiftSpeed <= 0) {
        targetLiftTime = Timer.getFPGATimestamp() + kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetLiftTime) {
        if (!m_joystick.getRawButton(4)) {
          m_leftLiftMtr.setVoltage(speed * kMaxVoltageLeft);
        } else {
          m_leftLiftMtr.setVoltage(0);
        }
        if (!m_joystick.getRawButton(3)) {
          m_rightLiftMtr.setVoltage(speed * kMaxVoltageRight);
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

    if (m_joystick.getRawButton(4)) {
      closedLoopMode = 0;
    }

    if (m_joystick.getRawButton(6)) {
      closedLoopMode = 1;
    }

    if (closedLoopMode == 0) { //Retracting
      //If dynamic kF enabled, use higher FF gain to counteract
      //weight of robot while lifting
      if (prevRatchet == false && isDynamicKF) {
        m_leftLiftPid.setFF(kFLoadedLift);
        m_rightLiftPid.setFF(kFLoadedLift);
      }

      //Set target position to retract and deploy ratchet
      m_leftLiftPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_rightLiftPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_leftAcuator.setSpeed(kRatchetDeploy);
      m_rightAcuator.setSpeed(kRatchetDeploy);
      prevRatchet = true;
    }


    if (closedLoopMode == 1) { //Extending
      //Use default FF gain when lifting (no weight to account for)
      if (prevRatchet == true && isDynamicKF) {
        m_leftLiftPid.setFF(kFUnloadedLift);
        m_rightLiftPid.setFF(kFUnloadedLift);
      }

      m_leftAcuator.setSpeed(kRatchetRetract);
      m_rightAcuator.setSpeed(kRatchetRetract);

      //Delay 1 second so ratchets can fully deploy
      if (prevRatchet == true) {
        targetLiftTime = Timer.getFPGATimestamp() + kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetLiftTime) {
        m_leftLiftPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        m_rightLiftPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("Run");
      } else {
        m_leftLiftMtr.setVoltage(0);
        m_rightLiftMtr.setVoltage(0);
      }
      prevRatchet = false;
    }
  }

}

// left ratchet = -0.3