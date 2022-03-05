// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Joystick m_joystick = new Joystick(0);
  private CANSparkMax m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(4, MotorType.kBrushless);
  private MecanumDrive m_Drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private static final double kP = 0.005;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private PIDController m_pidController = new PIDController(kP, kI, kD);

  private final double kMaxSpeed = 0.25;

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

    m_pidController.disableContinuousInput();

    SmartDashboard.putData("Drivetrain", m_Drive);
    SmartDashboard.putData("Gyro", m_gyro);
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
    m_Drive.driveCartesian(-m_joystick.getRawAxis(1) * kMaxSpeed,
      m_joystick.getRawAxis(0) * kMaxSpeed,
      m_joystick.getRawAxis(2) * kMaxSpeed);
  }

  public void gyroAssistDriveCartesian(double ySpeed, double xSpeed, double rot) {
    m_Drive.driveCartesian(ySpeed, xSpeed, rot);
  }

  public void gyroAssistDrivePolar() {

  }

  public void getDistance() {
    //Avg x dist, avg y
  }

}
