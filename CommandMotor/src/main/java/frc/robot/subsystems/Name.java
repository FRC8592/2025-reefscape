// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;

public class Name extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX Commandedmotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);
  private VoltageOut commandedMotorVolts = new VoltageOut(0);
  
  public Name() {
  
    TalonFXConfiguration TFXconfig = new TalonFXConfiguration();
    Commandedmotor.getConfigurator().apply(TFXconfig);

  }

/**
 * Sets output of motor to outputPercent
 * @param outputPercent a value from -1 to 1  
 */
  public void setMotorOutput (double outputPercent){
    double maxmotorvolts = Commandedmotor.getSupplyVoltage().getValueAsDouble();
    Commandedmotor.setControl(commandedMotorVolts.withOutput(maxmotorvolts * outputPercent));   

  }

}
