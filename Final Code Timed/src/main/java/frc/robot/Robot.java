// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private boolean isEvac = false;
  private boolean isLaunching = false;
  private boolean isIntaking = false;
  private boolean prevTrigger = false;
  private double lwrStorageTargetTime = 0;
  private double turnPID = 0;

Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.254);
Translation2d m_frontRightLocation = new Translation2d(0.254, -0.254);
Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.254);
Translation2d m_backRightLocation = new Translation2d(-0.254, -0.254);

// Creating my kinematics object using the wheel locations.
MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());
Pose2d m_pose = new Pose2d();
  
  private static final double kLwrStorageDelay = 1;
  private static final double kDriveGearing = 10.71;
  private static final double kWheelCircummference = 6 * Math.PI;
  private static final double intkVltge = 4;
  private static final double uprMtrSpd = 1;
  private static final double lwrMtrSpd = 0.6;
  private static final double kMaxDriveSpd = 0.75;
  private static final double kdrivedeadband = 0.1;
  private static final double kMaxTurn = 360;
  private static final double kP = 0.008;
  private static final double kD = 0.00005;
  private static final double kF = 0.1;


  private Joystick m_joystick = new Joystick(0);
  private Joystick m_controller = new Joystick(1);

  private WPI_VictorSPX m_uprStorageMtr = new WPI_VictorSPX(7);
  private WPI_VictorSPX m_lwrStorageMtr = new WPI_VictorSPX(5);

  private CANSparkMax m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax m_intakeMotor = new CANSparkMax(6, MotorType.kBrushless);

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_backLeftEncoder;
  private RelativeEncoder m_backRightEncoder;

  private MecanumDrive m_drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private PIDController m_TurnPID = new PIDController(kP, 0, kD);
  private PowerDistribution m_pdh = new PowerDistribution();
  

  enum DriveMode{
    DEFAULT,
    FIELD_CENTRIC,
    GYRO_ASSIST,
    GYRO_ASSIST_FIELD_CENTER
  }

  private static final DriveMode mode = DriveMode.DEFAULT;
 
  @Override
  public void robotInit() {
   m_lwrStorageMtr.configFactoryDefault();
   m_uprStorageMtr.configFactoryDefault();
   m_leftBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    

    m_frontLeftEncoder = m_leftFront.getEncoder();
    m_frontRightEncoder = m_rightFront.getEncoder();
    m_backLeftEncoder = m_leftBack.getEncoder();
    m_backRightEncoder = m_rightBack.getEncoder();

    m_frontLeftEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_frontRightEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_backLeftEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_backRightEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);

    m_frontLeftEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_frontRightEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_backLeftEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_backRightEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);

    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);
    m_lwrStorageMtr.setInverted(true);

    m_gyro.calibrate();
    m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    m_TurnPID.disableContinuousInput();

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    CameraServer.startAutomaticCapture(2);

  }

  @Override
  public void robotPeriodic() {
    if (m_joystick.getTrigger()) {
      isLaunching = true;
    } else {
      isLaunching = false;
    }
    if (m_joystick.getRawButton(2)) {
      isIntaking = true;
    } else {
      isIntaking = false;
    }
    if (m_joystick.getRawButton(7)) {
      isEvac = true;
    } else {
      isEvac = false;
    }
    

    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putData("drive", m_drive);
    SmartDashboard.putNumber("pose X", m_pose.getX());
    SmartDashboard.putNumber("pose Y", m_pose.getY());
    SmartDashboard.putNumber("PID Out", turnPID);

    double avgDriveCurrent = (m_pdh.getCurrent(0) + m_pdh.getCurrent(0) + m_pdh.getCurrent(0) + m_pdh.getCurrent(0)) / 4;

    var wheelSpeeds = new MecanumDriveWheelSpeeds(
      Units.inchesToMeters(m_frontLeftEncoder.getVelocity()), Units.inchesToMeters(m_frontRightEncoder.getVelocity()),
      Units.inchesToMeters(m_backLeftEncoder.getVelocity()), Units.inchesToMeters(m_backRightEncoder.getVelocity()));

  // Get my gyro angle. We are negating the value because gyros return positive
  // values as the robot turns clockwise. This is not standard convention that is
  // used by the WPILib classes.
  var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

  // Update the pose
  m_pose = m_odometry.update(gyroAngle, wheelSpeeds);
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

    double yspeed = -m_controller.getRawAxis(1) * Math.abs(m_controller.getRawAxis(1));
    double xspeed = m_controller.getRawAxis(0) * Math.abs(m_controller.getRawAxis(0));
    double zrot = m_controller.getRawAxis(2) * Math.abs(m_controller.getRawAxis(2));

    if (Math.abs(yspeed) < kdrivedeadband) yspeed = 0;
    if (Math.abs(xspeed) < kdrivedeadband) xspeed = 0;
    if (Math.abs(zrot) < kdrivedeadband) zrot = 0;

    yspeed *= kMaxDriveSpd;
    xspeed *= kMaxDriveSpd;
    zrot *= kMaxDriveSpd;

    turnPID = m_TurnPID.calculate(m_gyro.getRate(), zrot * kMaxTurn);
    if (turnPID > 0) {
      turnPID -= kF;
    } else if (turnPID < 0) {
      turnPID += kF;
    }

    if (mode == DriveMode.DEFAULT) {
      m_drive.driveCartesian(yspeed, xspeed, zrot);
    } else if (mode == DriveMode.FIELD_CENTRIC){
      m_drive.driveCartesian(yspeed, xspeed, zrot, m_gyro.getAngle());
    } else if (mode == DriveMode.GYRO_ASSIST) {

    } else if (mode == DriveMode.GYRO_ASSIST_FIELD_CENTER) {

    }

    if(!isEvac) {
      m_intakeMotor.setVoltage(0);
      m_lwrStorageMtr.set(0);
      m_uprStorageMtr.set(0);
    }

    if (isIntaking) {
      m_lwrStorageMtr.set(ControlMode.PercentOutput, lwrMtrSpd);
      m_intakeMotor.setVoltage(intkVltge);
    } else {
      m_lwrStorageMtr.set(ControlMode.PercentOutput, 0);
      m_intakeMotor.setVoltage(0);
    }

    if (isLaunching ) {
      m_uprStorageMtr.set(ControlMode.PercentOutput, uprMtrSpd);
      if (! prevTrigger){
        lwrStorageTargetTime = Timer.getFPGATimestamp()+kLwrStorageDelay;
      }

      if (Timer.getFPGATimestamp() > lwrStorageTargetTime){
        m_lwrStorageMtr.set(ControlMode.PercentOutput, lwrMtrSpd);
      } else m_lwrStorageMtr.set(ControlMode.PercentOutput, 0);
    } else {
      m_uprStorageMtr.set(ControlMode.PercentOutput, 0);
    }

    if(isEvac) {
      m_intakeMotor.setVoltage(-intkVltge);
      m_lwrStorageMtr.set(-lwrMtrSpd);
      m_uprStorageMtr.set(-lwrMtrSpd);
    }

    prevTrigger = m_joystick.getTrigger();
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
