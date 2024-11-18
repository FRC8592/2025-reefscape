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
  private TalonFX testMotor;
  private TalonFXConfiguration config;
  private VoltageOut motorVoltage;
  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    testMotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);
    config = new TalonFXConfiguration();
    
    testMotor.getConfigurator().apply(config);

    motorVoltage = new VoltageOut(0);

  }

  public double getMaxMotorVoltage (){

    return testMotor.getSupplyVoltage().getValueAsDouble();
  }

  /**
   * Driving the motor at a certain percentage
   * @param joystickValue 
   */
  public void setPercentMotorOutput (double joystickValue){
    //Multiplying the Joystick value to the Motor voltage
    testMotor.setControl(motorVoltage.withOutput(getMaxMotorVoltage()*joystickValue));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
