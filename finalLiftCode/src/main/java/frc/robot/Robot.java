// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {
  
  //Constant for motion magic
  private static final double kP = 0.00015;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kFUnloaded = 0;
  private static final double kFLoaded = 0.0004;
  private static final double kMinOutput = -1;
  private static final double kMaxOutput = 1;
  private static final double maxVel = 2000; // rpm
  private static final double maxAcc = 1500;
  private static final double allowedErr = 0;

  private static final boolean isPid = true; //Run in PID mode instead of direct control (open loop)
  private static final boolean isDynamicKF = false; //EXPERIMENTAL. USE WITH CAUTION
  private static final double kLowSetpoint = 0; //Units: motor rotations
  private static final double kHighSetpoint = 20; //Units: motor rotations
  private static final double kMaxVoltage = 4;
  private static final double kRatchetDeploy = 1;
  private static final double kRatchetRetract = -1;
  private static final double kRatchetDelay = 1;

  private static final int smartMotionSlot = 0;
  private static final int kLeftLiftID = 4;
  private static final int kRightLiftID = 5;
  private static final int kJoystickPort = 0;
  private static final int kLeftServoPort = 0;
  private static final int kRightServoPort = 1;

  private CANSparkMax m_leftLift = new CANSparkMax(kLeftLiftID, MotorType.kBrushless);
  private CANSparkMax m_rightLift = new CANSparkMax(kRightLiftID, MotorType.kBrushless);
  private SparkMaxPIDController m_leftPid;
  private SparkMaxPIDController m_rightPid;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private Joystick m_joystick = new Joystick(kJoystickPort);

  private Servo m_leftAcuator = new Servo(kLeftServoPort);
  private Servo m_rightAcuator = new Servo(kRightServoPort);

  private double targetTime = 0;
  private double prevLiftSpeed = 0;
  private boolean prevRatchet = true;
  private int closedLoopMode = 0; // 1: High, 0: Low

  @Override
  public void robotInit() {
    m_leftLift.restoreFactoryDefaults();
    m_rightLift.restoreFactoryDefaults();

    m_leftLift.setInverted(false);
    m_rightLift.setInverted(true);

    m_leftAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_rightAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    m_rightEncoder = m_rightLift.getEncoder();
    m_rightPid = m_rightLift.getPIDController();
    m_rightPid.setP(kP);
    m_rightPid.setI(kI);
    m_rightPid.setD(kD);
    m_rightPid.setIZone(0);
    m_rightPid.setFF(kFUnloaded);
    m_rightPid.setOutputRange(kMinOutput, kMaxOutput);

    m_rightPid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rightPid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rightPid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_leftEncoder = m_leftLift.getEncoder();
    m_leftPid = m_leftLift.getPIDController();
    m_leftPid.setP(kP);
    m_leftPid.setI(kI);
    m_leftPid.setD(kD);
    m_leftPid.setIZone(0);
    m_leftPid.setFF(kFUnloaded);
    m_leftPid.setOutputRange(kMinOutput, kMaxOutput);

    m_leftPid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_leftPid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_leftPid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    //Deploy ratchets at match start
    m_leftAcuator.setSpeed(kRatchetDeploy);
    m_rightAcuator.setSpeed(kRatchetDeploy);
  }


  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightEncoder.getPosition());
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
        m_leftLift.setVoltage(0);
        m_rightLift.setVoltage(0);
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
  public void testPeriodic() {}

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

    if (speed <= 0) { //Retracting Lift
      m_leftAcuator.setSpeed(kRatchetDeploy);
      m_rightAcuator.setSpeed(kRatchetDeploy);

      m_leftLift.setVoltage(speed * kMaxVoltage);
      m_rightLift.setVoltage(speed * kMaxVoltage);
    } else { //Raising Lift
      //Ratchets must be fully retracted to raise lift arms
      m_leftAcuator.setSpeed(kRatchetRetract);
      m_rightAcuator.setSpeed(kRatchetRetract);

      //Allow arms to move after 1 second has passed
      if (prevLiftSpeed <= 0) {
        targetTime = Timer.getFPGATimestamp() + kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetTime) {
        m_leftLift.setVoltage(speed * kMaxVoltage);
        m_rightLift.setVoltage(speed * kMaxVoltage);
      } else {
        m_leftLift.setVoltage(0);
        m_rightLift.setVoltage(0);
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
        m_leftPid.setFF(kFLoaded);
        m_rightPid.setFF(kFLoaded);
      }

      //Set target position to retract and deploy ratchet
      m_leftPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_rightPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_leftAcuator.setSpeed(kRatchetDeploy);
      m_rightAcuator.setSpeed(kRatchetDeploy);
      prevRatchet = true;
    }


    if (closedLoopMode == 1) { //Extending
      //Use default FF gain when lifting (no weight to account for)
      if (prevRatchet == true && isDynamicKF) {
        m_leftPid.setFF(kFUnloaded);
        m_rightPid.setFF(kFUnloaded);
      }

      m_leftAcuator.setSpeed(kRatchetRetract);
      m_rightAcuator.setSpeed(kRatchetRetract);

      //Delay 1 second so ratchets can fully deploy
      if (prevRatchet == true) {
        targetTime = Timer.getFPGATimestamp() + kRatchetDelay;
      }
      if (Timer.getFPGATimestamp() >= targetTime) {
        m_leftPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        m_rightPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
        System.out.println("Run");
      } else {
        m_leftLift.setVoltage(0);
        m_rightLift.setVoltage(0);
      }
      prevRatchet = false;
    }
  }

}

// add limit switches?
