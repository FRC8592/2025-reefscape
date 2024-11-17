// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.beans.DesignMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;

public class DriveTrain extends SubsystemBase {
  TalonFX testMotor;
  VoltageOut desiredMotorVoltage = new VoltageOut(0);

  public DriveTrain() {
    //Setting factory defaults for the motorz
    TalonFXConfiguration factoryConfigs = new TalonFXConfiguration();

    //Initializing the test motor
    TalonFX testMotor = new TalonFX(Constants.MAIN_MOTOR_CAN_ID);
    testMotor.getConfigurator().apply(factoryConfigs);
  }

  /**
   * Get the actual voltage available at the motor input
   * @return
   */
  public double getMaxMotorVoltage() {
    return testMotor.getSupplyVoltage().getValueAsDouble();
  }

  /**
   * Set the motor percent output
   * @param percentPower This parameter is for the power inputted into the motor in percentage from -1 to 1
   */
  public void setMotorPercentOutput(double percentPower) {
    testMotor.setControl(desiredMotorVoltage.withOutput(percentPower * getMaxMotorVoltage()));
  }

}
