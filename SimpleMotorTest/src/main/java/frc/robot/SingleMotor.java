package frc.robot;

import com.ctre.phoenix6.hardware.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public class SingleMotor {
    private TalonFX testMotor;
    VoltageOut testMotorVoltage = new VoltageOut(0); // Datatype needed to control voltage

    public SingleMotor () {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // Create the controller object for our test motor
        testMotor = new TalonFX(Constants.MOTOR_CAN_ID);

        // Set factory defaults for all motor configuration options
        testMotor.getConfigurator().apply(talonFXConfigs);

        // Set the motor to zero power for safety
        testMotor.setControl(testMotorVoltage.withOutput(0));
    }

    /**
     * Set the motor power in terms of percent
     * @param outputPercent Specify motor power in a range of -1 to +1
     */
    public void setPercentMotorOutput(double outputPercent) {
        double motorVoltage = testMotor.getSupplyVoltage().getValueAsDouble();
    
        testMotor.setControl(testMotorVoltage.withOutput(motorVoltage * outputPercent));
    }

}