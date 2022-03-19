// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeedbackSubsystem extends SubsystemBase {

  private ADIS16470_IMU m_gyro;
  private PowerDistribution m_pdh;
  private XboxController m_controller;

  private double netAccelertion = 0;

  /** Creates a new FeedbackSystem. */
  public FeedbackSubsystem(ADIS16470_IMU gyro, PowerDistribution pdh, XboxController controller) {

    m_gyro = gyro;
    m_pdh = pdh;
    m_controller = controller;

    var cam0 = CameraServer.startAutomaticCapture(0);
    var cam1 = CameraServer.startAutomaticCapture(1);
    var cam2 = CameraServer.startAutomaticCapture(2);

    cam0.setResolution(120, 120);
    cam1.setResolution(120, 120);
    cam2.setResolution(360, 240);

    cam0.setFPS(15);
    cam1.setFPS(10);
    cam2.setFPS(30);
  }

  @Override
  public void periodic() {
    // Calculate net acceleration (all directions) and subtract acc due to gravity
    netAccelertion = Math.sqrt(Math.pow(m_gyro.getAccelX(), 2) +
      Math.pow(m_gyro.getAccelY(), 2) + Math.pow(m_gyro.getAccelZ(), 2))-9.8;
    netAccelertion = Math.abs(netAccelertion);

    // Deadband net acceleration for random fluctuations
    if (netAccelertion < 0.05) {
      netAccelertion = 0; 
    }

    // Calculate avg current from drive motors only
    double avgDriveCurrent = ( m_pdh.getCurrent(Constants.Feedback.kDrivePowerChannels[0]) +
                          m_pdh.getCurrent(Constants.Feedback.kDrivePowerChannels[1]) +
                          m_pdh.getCurrent(Constants.Feedback.kDrivePowerChannels[2]) +
                          m_pdh.getCurrent(Constants.Feedback.kDrivePowerChannels[3])
                        ) / 4;
    double currentRatio = (avgDriveCurrent / netAccelertion);

    // Pulse if robot is pushing against obstacle or other robot
    boolean isPulsing;
    if (currentRatio > Constants.Feedback.kCurrentRatioRumbleThreshold) {
      isPulsing = true;
    } else {
      isPulsing = false;
    }

    //If pulsing, alternate between full rumble and no rumble
    if (isPulsing) {
      double pulseTime = (Timer.getFPGATimestamp() * Constants.Feedback.kRumblePulseRate) % 1;

      if (pulseTime < Constants.Feedback.kRumblePulseWidth) {
        m_controller.setRumble(RumbleType.kLeftRumble, Constants.Feedback.kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, Constants.Feedback.kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, -Constants.Feedback.kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, -Constants.Feedback.kRumbleStrength);
      }
    //Otherwise rumble when past the acceleration threshold
    } else {
      if (netAccelertion > Constants.Feedback.kAccelerationRumbleThreshold) {
        m_controller.setRumble(RumbleType.kLeftRumble, Constants.Feedback.kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, Constants.Feedback.kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
      }
    }
  }
}
