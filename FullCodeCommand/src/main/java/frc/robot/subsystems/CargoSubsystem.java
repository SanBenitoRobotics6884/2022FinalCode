// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public CargoSubsystem() {
    m_lowerStorageMotor.configFactoryDefault();
    m_upperStorageMotor.configFactoryDefault();
    m_intakeMotor.restoreFactoryDefaults();

    m_lowerStorageMotor.setInverted(true);
    m_upperStorageMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
  }

  public void runIntake(double voltage) {
    m_intakeMotor.setVoltage(voltage);
  }

  public void runLowerStorage(double speed) {
    m_lowerStorageMotor.set(speed);
  }

  public void runUpperStorage(double speed) {
    m_upperStorageMotor.set(speed);
  }
  

}
