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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.254);
  private Translation2d m_frontRightLocation = new Translation2d(0.254, -0.254);
  private Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.254);
  private Translation2d m_backRightLocation = new Translation2d(-0.254, -0.254);

  private MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  private MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());
  private Pose2d m_pose = new Pose2d();

  private CANSparkMax m_leftFront = new CANSparkMax(Constants.Drive.kLeftFrontMotor, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(Constants.Drive.kRightFrontMotor, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(Constants.Drive.kLeftBackMotor, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(Constants.Drive.kRightBackMotor, MotorType.kBrushless);

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_backLeftEncoder;
  private RelativeEncoder m_backRightEncoder;

  private MecanumDrive m_drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
  private ADIS16470_IMU m_gyro;
  private PIDController m_TurnPID = new PIDController(Constants.Drive.TurnRatePID.kP,
                                                      Constants.Drive.TurnRatePID.kI, 
                                                      Constants.Drive.TurnRatePID.kD);

  private double turnPID = 0;
  private double maxDriveSpdScalar = Constants.Drive.kSlowSpd;

  public enum DriveMode{
    DEFAULT,
    FIELD_CENTRIC,
    GYRO_ASSIST,
    GYRO_ASSIST_FIELD_CENTER
  }

  private DriveMode mode = DriveMode.DEFAULT;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem(ADIS16470_IMU gyro) {

    m_gyro = gyro;

    m_leftBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();

    m_frontLeftEncoder = m_leftFront.getEncoder();
    m_frontRightEncoder = m_rightFront.getEncoder();
    m_backLeftEncoder = m_leftBack.getEncoder();
    m_backRightEncoder = m_rightBack.getEncoder();

    double kConversionFactor = Constants.Drive.kWheelCircumference / Constants.Drive.kDriveGearing;
    m_frontLeftEncoder.setVelocityConversionFactor(kConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(kConversionFactor);
    m_backLeftEncoder.setVelocityConversionFactor(kConversionFactor);
    m_backRightEncoder.setVelocityConversionFactor(kConversionFactor);

    m_frontLeftEncoder.setPositionConversionFactor(kConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(kConversionFactor);
    m_backLeftEncoder.setPositionConversionFactor(kConversionFactor);
    m_backRightEncoder.setPositionConversionFactor(kConversionFactor);

    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);

    m_TurnPID.disableContinuousInput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      Units.inchesToMeters(m_frontLeftEncoder.getVelocity()), Units.inchesToMeters(m_frontRightEncoder.getVelocity()),
      Units.inchesToMeters(m_backLeftEncoder.getVelocity()), Units.inchesToMeters(m_backRightEncoder.getVelocity()));

    Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle()); // clockwise should be negative

    m_pose = m_odometry.update(gyroAngle, wheelSpeeds);

    /*
    double yspeed = -inputProcess(m_controller.getLeftY(), kdrivedeadband, maxDriveSpdScalar);
    double xspeed = inputProcess(m_controller.getLeftX(), kdrivedeadband, maxDriveSpdScalar);
    double zrot = inputProcess(m_controller.getRightX(), kdrivedeadband, maxDriveSpdScalar);
    */
    
  }

  @Override
  public void simulationPeriodic() { }

  public void drive(double forw, double strafe, double rot) {
    double yspeed = -inputProcess(forw, Constants.Drive.kdrivedeadband, maxDriveSpdScalar, true);
    double xspeed = inputProcess(strafe, Constants.Drive.kdrivedeadband, maxDriveSpdScalar, true);
    double zrot = inputProcess(rot, Constants.Drive.kdrivedeadband, maxDriveSpdScalar, true);

    if (mode == DriveMode.DEFAULT) {
      m_drive.driveCartesian(yspeed, xspeed, zrot);
    } else if (mode == DriveMode.FIELD_CENTRIC){
      m_drive.driveCartesian(yspeed, xspeed, zrot, m_gyro.getAngle());
    } else if (mode == DriveMode.GYRO_ASSIST) {
      turnPID = m_TurnPID.calculate(m_gyro.getRate(), zrot * Constants.Drive.kMaxTurn);
      if (turnPID > 0) {
        turnPID -= Constants.Drive.TurnRatePID.kF;
      } else if (turnPID < 0) {
        turnPID += Constants.Drive.TurnRatePID.kF;
      }
    } else if (mode == DriveMode.GYRO_ASSIST_FIELD_CENTER) {
      // gyro turn rate assist plus field centric mode
    }
  }

  private double inputProcess(double input, double deadband, double maxValue, boolean squareInputs) {
    double processed = input;
    if (squareInputs) processed = processed * Math.abs(processed);
    if (Math.abs(processed) < deadband) processed = 0;
    processed *= maxValue;
    return processed;
  }

  public void setDriveMode(DriveMode driveMode) {
    mode = driveMode;
  }

  public DriveMode getDriveMode() {
    return mode;
  }

  public  void setMaxSpeed(double speed) {
    maxDriveSpdScalar = speed;
  }

  public double getMaxSpeed(double speed) {
    return maxDriveSpdScalar;
  }

  public Pose2d getRobotPose() {
    return m_pose;
  }

}
