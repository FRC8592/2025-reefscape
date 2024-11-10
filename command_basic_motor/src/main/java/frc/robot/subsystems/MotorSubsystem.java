// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;

public class MotorSubsystem extends SubsystemBase {
  //
  // Motor objects
  //
  private TalonFX mainMotor;
  private VoltageOut mainMotorVoltage = new VoltageOut(0);


  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    TalonFXConfiguration mainMotorConfig = new TalonFXConfiguration();

    // Instantiate the main motor for testing and reset it to factory defaults
    mainMotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);
    mainMotor.getConfigurator().apply(mainMotorConfig);
  }

  /**
   * Sets the operating voltage for the main motor
   * @param voltage double specifying the drive voltage for the motor
   */
  public void setMotorVoltage(double voltage) {
    this.mainMotor.setControl(mainMotorVoltage.withOutput(voltage));
  }

  /**
   * 
   * @return Voltage at motor inputs
   */
  public double getAvailableVoltage() {
    return mainMotor.getSupplyVoltage().getValueAsDouble();
  }

}
