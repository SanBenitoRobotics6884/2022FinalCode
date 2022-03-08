// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.254);
  Translation2d m_frontRightLocation = new Translation2d(0.254, -0.254);
  Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.254);
  Translation2d m_backRightLocation = new Translation2d(-0.254, -0.254);

  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());
  Pose2d m_pose = new Pose2d();

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

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
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

    m_gyro.calibrate();
    m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    m_TurnPID.disableContinuousInput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      Units.inchesToMeters(m_frontLeftEncoder.getVelocity()), Units.inchesToMeters(m_frontRightEncoder.getVelocity()),
      Units.inchesToMeters(m_backLeftEncoder.getVelocity()), Units.inchesToMeters(m_backRightEncoder.getVelocity()));

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

    // Update the pose
    m_pose = m_odometry.update(gyroAngle, wheelSpeeds);

    double yspeed = -m_controller.getLeftY() * Math.abs(m_controller.getLeftY());
    double xspeed = m_controller.getLeftX() * Math.abs(m_controller.getLeftX());
    double zrot = m_controller.getRightX() * Math.abs(m_controller.getRightX());

    if (Math.abs(yspeed) < kdrivedeadband) yspeed = 0;
    if (Math.abs(xspeed) < kdrivedeadband) xspeed = 0;
    if (Math.abs(zrot) < kdrivedeadband) zrot = 0;

    yspeed *= maxDriveSpdScalar;
    xspeed *= maxDriveSpdScalar;
    zrot *= maxDriveSpdScalar;

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
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
