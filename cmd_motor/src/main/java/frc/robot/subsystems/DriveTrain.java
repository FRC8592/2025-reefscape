// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class DriveTrain extends SubsystemBase {
  private TalonFX motor;
  private VoltageOut motorVoltage = new VoltageOut(0);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    motor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    motor.getConfigurator().apply(talonFXConfigs);
    motor.setControl(motorVoltage.withOutput(0));

  }

  public double getMaxMotorVoltage() {
    return motor.getSupplyVoltage().getValueAsDouble();
  }

  /**
   * sets the motor output 
   * @param joystickValue negative -1 to 1 from joystick movement
   */
  public void setMotorOutput(double joystickValue){
    motor.setControl(motorVoltage.withOutput(getMaxMotorVoltage() * joystickValue));
  }

}
