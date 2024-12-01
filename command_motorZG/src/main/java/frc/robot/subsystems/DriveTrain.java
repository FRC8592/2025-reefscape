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

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX mainMotor;
  VoltageOut desiredMotorVoltage = new VoltageOut(0);

  public DriveTrain() {
    TalonFXConfiguration factoryConfig = new TalonFXConfiguration();

    //Instantiate the motor and set factory default
    mainMotor = new TalonFX(Constants.MAIN_MOTOR_CAN_ID);
    mainMotor.getConfigurator().apply(factoryConfig);

  }

  public double getMaxMotorVoltage () {
    return mainMotor.getSupplyVoltage().getValueAsDouble();
  }

  public void setMotorPercentOutput(double percentPower) {
    mainMotor.setControl(desiredMotorVoltage.withOutput(percentPower*getMaxMotorVoltage()));
  }



}
