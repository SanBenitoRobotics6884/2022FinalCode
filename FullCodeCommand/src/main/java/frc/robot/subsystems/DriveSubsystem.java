// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    Constants.Drive.m_frontLeftLocation,
    Constants.Drive.m_frontRightLocation,
    Constants.Drive.m_backLeftLocation,
    Constants.Drive.m_backRightLocation);
  
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
  private PIDController m_TurnPID = new PIDController
    (Constants.Drive.TurnRatePID.kP,
    Constants.Drive.TurnRatePID.kI, 
    Constants.Drive.TurnRatePID.kD);

  private TrapezoidProfile.Constraints m_positionConstraints
    = new TrapezoidProfile.Constraints(Constants.Drive.PositionPID.kMaxVel, Constants.Drive.PositionPID.kMaxAcc);

  private ProfiledPIDController m_positionControllerY = new ProfiledPIDController
    (Constants.Drive.PositionPID.kP,
    Constants.Drive.PositionPID.kI,
    Constants.Drive.PositionPID.kD,
    m_positionConstraints);

  private ProfiledPIDController m_positionControllerX = new ProfiledPIDController
    (Constants.Drive.PositionPID.kP,
    Constants.Drive.PositionPID.kI,
    Constants.Drive.PositionPID.kD,
    m_positionConstraints);

  private TrapezoidProfile.Constraints m_angleConstraints
    = new TrapezoidProfile.Constraints(Constants.Drive.AbsoluteAnglePID.kMaxVelRot, Constants.Drive.AbsoluteAnglePID.kMaxAccRot);

  private ProfiledPIDController m_positionControllerAngle = new ProfiledPIDController
    (Constants.Drive.AbsoluteAnglePID.kP,
    Constants.Drive.AbsoluteAnglePID.kI,
    Constants.Drive.AbsoluteAnglePID.kD,
    m_angleConstraints);

    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(Constants.Drive.kRateLimit);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(Constants.Drive.kRateLimit);

  private double turnPID = 0;
  private double angleSetpoint = 0;
  private double maxDriveSpdScalar = Constants.Drive.kSlowSpd;
  private double maxTurnSpdScalar = Constants.Drive.kDefaultTurn;

  public enum DriveMode{
    DEFAULT,
    FIELD_CENTRIC,
    GYRO_ASSIST,
    GYRO_ASSIST_FIELD_CENTER,
    EXPERIMENTAL,
    EXPERIMENTALGYROASSIST
  }

  private DriveMode mode = DriveMode.GYRO_ASSIST;

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

    m_frontLeftEncoder.setVelocityConversionFactor(Constants.Drive.kConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(Constants.Drive.kConversionFactor);
    m_backLeftEncoder.setVelocityConversionFactor(Constants.Drive.kConversionFactor);
    m_backRightEncoder.setVelocityConversionFactor(Constants.Drive.kConversionFactor);

    m_frontLeftEncoder.setPositionConversionFactor(Constants.Drive.kConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(Constants.Drive.kConversionFactor);
    m_backLeftEncoder.setPositionConversionFactor(Constants.Drive.kConversionFactor);
    m_backRightEncoder.setPositionConversionFactor(Constants.Drive.kConversionFactor);

    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);

    m_TurnPID.disableContinuousInput();

    SmartDashboard.putData(m_drive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      Units.inchesToMeters(m_frontLeftEncoder.getVelocity()), Units.inchesToMeters(m_frontRightEncoder.getVelocity()),
      Units.inchesToMeters(m_backLeftEncoder.getVelocity()), Units.inchesToMeters(m_backRightEncoder.getVelocity()));

    Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle()); // clockwise should be negative

    m_pose = m_odometry.update(gyroAngle, wheelSpeeds);

    SmartDashboard.putNumber("Pose X", m_pose.getX());
    SmartDashboard.putNumber("Pose Y", m_pose.getY());
    SmartDashboard.putNumber("Angle", m_gyro.getAngle());
  }

  @Override
  public void simulationPeriodic() { }

  public void drive(double forw, double strafe, double rot) {
    double yspeed = -inputProcess(forw, Constants.Drive.kdrivedeadband, maxDriveSpdScalar, false);
    double xspeed = inputProcess(strafe, Constants.Drive.kdrivedeadband, maxDriveSpdScalar, false);
    double zrot = inputProcess(rot, Constants.Drive.kdrivedeadband, maxTurnSpdScalar, false);

    if (mode == DriveMode.GYRO_ASSIST || mode == DriveMode.GYRO_ASSIST_FIELD_CENTER) {
      turnPID = m_TurnPID.calculate(m_gyro.getRate(), zrot * Constants.Drive.kMaxTurn);
    } else if (mode == DriveMode.EXPERIMENTAL || mode == DriveMode.EXPERIMENTALGYROASSIST) {
      angleSetpoint += zrot * Constants.Drive.kMaxTurn * 0.02;
      turnPID = m_positionControllerAngle.calculate(m_gyro.getAngle(), angleSetpoint);
    }

    if (turnPID > 0) {
      turnPID += Constants.Drive.kFTurn;
    } else if (turnPID < 0) {
      turnPID -= Constants.Drive.kFTurn;
    }

    yspeed = m_yLimiter.calculate(yspeed);
    if (maxDriveSpdScalar >= Constants.Drive.kFastSpd) {
      xspeed *= Constants.Drive.kAdditionalFastStrafeMultiplier;
    }

    if (mode == DriveMode.DEFAULT) {
      m_drive.driveCartesian(yspeed, xspeed, zrot);
    } else if (mode == DriveMode.FIELD_CENTRIC){
      m_drive.driveCartesian(yspeed, xspeed, zrot, m_gyro.getAngle());
    } else if (mode == DriveMode.GYRO_ASSIST || mode == DriveMode.EXPERIMENTAL) {
      m_drive.driveCartesian(yspeed, xspeed, inputProcess(turnPID, 0.05, 1, false));
    } else if (mode == DriveMode.GYRO_ASSIST_FIELD_CENTER || mode == DriveMode.EXPERIMENTALGYROASSIST) {
      m_drive.driveCartesian(yspeed, xspeed, inputProcess(turnPID, 0.05, 1, false), m_gyro.getAngle());
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
    angleSetpoint = m_gyro.getAngle();
    mode = driveMode;
  }

  public void setPositionTarget(double xDist, double yDist, double zRot) {

    m_positionControllerY.setGoal(yDist);
    m_positionControllerX.setGoal(xDist);
    m_positionControllerAngle.setGoal(zRot);
  }

  public void driveTowardTarget() {

    double strafe = -m_positionControllerX.calculate(m_pose.getY());
    double forw = m_positionControllerY.calculate(m_pose.getX());
    double rot = m_positionControllerAngle.calculate(m_gyro.getAngle());

    if (rot > 0) {
      rot += Constants.Drive.AbsoluteAnglePID.kF;
    } else if (rot < 0) {
      rot -= Constants.Drive.AbsoluteAnglePID.kF;
    }

    if (forw > 0) {
      forw += Constants.Drive.PositionPID.kF;
    } else if (forw < 0) {
      forw -= Constants.Drive.PositionPID.kF;
    }

    if (strafe > 0) {
      strafe += Constants.Drive.PositionPID.kF;
    } else if (strafe < 0) {
      strafe -= Constants.Drive.PositionPID.kF;
    }
    
    m_drive.driveCartesian(
      forw,
      strafe,
      rot,
      m_gyro.getAngle());
  }

  public void calibrateGyro() {
    if (DriverStation.isDisabled()) {
      m_gyro.calibrate();
    }
  }

  public boolean atTargetPosition(double yTarget, double xTarget, double angleTarget) {
    boolean atY = Math.abs(yTarget - m_pose.getY()) < Constants.Drive.PositionPID.kAllowedError;
    boolean atX = Math.abs(xTarget - m_pose.getX()) < Constants.Drive.PositionPID.kAllowedError;
    boolean atZ = Math.abs(angleTarget - m_gyro.getAngle()) < Constants.Drive.AbsoluteAnglePID.kAllowedError;
    
    if (atY && atX && atZ) {
      drive(0,0,0);
      return true;
    }
    return false;
  }

  public DriveMode getDriveMode() {
    return mode;
  }

  public  void setMaxDriveSpeed(double driveSpeed) {
    maxDriveSpdScalar = driveSpeed;
  }
  public  void setMaxTurnSpeed(double turnSpeed) {
    maxTurnSpdScalar = turnSpeed;
  }


  public double getMaxSpeed(double speed) {
    return maxDriveSpdScalar;
  }

  public Pose2d getRobotPose() {
    return m_pose;
  }

  public double getRobotAngle() {
    return m_gyro.getAngle();
  }

  public void resetPose() {
    if (DriverStation.isDisabled()) {
      m_odometry.resetPosition(new Pose2d(), new Rotation2d());
    }
  }

}
