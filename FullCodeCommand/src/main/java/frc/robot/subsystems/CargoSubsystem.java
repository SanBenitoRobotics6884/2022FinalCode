// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CargoSubsystem extends SubsystemBase {

  private WPI_VictorSPX m_upperStorageMotor = new WPI_VictorSPX(Constants.Cargo.kUpperStorageID);
  private WPI_VictorSPX m_lowerStorageMotor = new WPI_VictorSPX(Constants.Cargo.kLowerStorageID);
  private CANSparkMax m_intakeMotor = new CANSparkMax(Constants.Cargo.kIntakeID, MotorType.kBrushless);

  private boolean isIntaking = false;
  private boolean isLaunching = false;
  private boolean isEvac = false;

  public CargoSubsystem() {
    m_lowerStorageMotor.configFactoryDefault();
    m_upperStorageMotor.configFactoryDefault();
    m_intakeMotor.restoreFactoryDefaults();

    m_lowerStorageMotor.setInverted(true);
    m_upperStorageMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    if (isIntaking) {
      m_lowerStorageMotor.set(ControlMode.PercentOutput, Constants.Cargo.kLwrMtrSpd);
      m_intakeMotor.setVoltage(Constants.Cargo.kIntkVltge);
    } else {
      m_lowerStorageMotor.set(ControlMode.PercentOutput, 0);
      m_intakeMotor.setVoltage(0);
    }

    if (isLaunching ) {
      m_upperStorageMotor.set(ControlMode.PercentOutput, Constants.Cargo.kLaunchSpd);
      m_lowerStorageMotor.set(Constants.Cargo.kLwrMtrSpd);
    } else {
      m_upperStorageMotor.set(ControlMode.PercentOutput, 0);
    }

    if(isEvac) {
      m_intakeMotor.setVoltage(-Constants.Cargo.kIntkVltge);
      m_lowerStorageMotor.set(-Constants.Cargo.kLwrMtrSpd);
      m_upperStorageMotor.set(-Constants.Cargo.kLwrMtrSpd);
    }
  }

  public void setIntakeStatus(boolean isRunning) {
    isIntaking = isRunning;
  }

  public void setLaunchStatus(boolean isRunning) {
    isLaunching = isRunning;
  }

  public void setEvacStatus(boolean isRunning) {
    isEvac = isRunning;
  }

  public boolean getIntakeStatus() {
    return isIntaking;
  }
  public boolean getLaunchStatus() {
    return isLaunching;
  }
  public boolean getEvacStatus() {
    return isEvac;
  }
  

}
