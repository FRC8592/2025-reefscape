package frc.robot;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public class TestMotor {
    
    private TalonFX testMotor;
    private VoltageOut testMotorVoltage = new VoltageOut(0);

    public TestMotor() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        testMotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);

        //sets motor to factory default configuration
        testMotor.getConfigurator().apply(talonFXConfigs);

        //sets motor to an initial power of 0 for safety
        testMotor.setControl(testMotorVoltage.withOutput(0));

    }

    /**
     * Sets the motor power using values from -1 to 1
     * @param outputJoystick
     */
    public void setMotorOutput(double outputJoystick) {

        //takes available voltage of the motor
        double motorVoltage = testMotor.getSupplyVoltage().getValueAsDouble();
        
        testMotor.setControl(testMotorVoltage.withOutput(outputJoystick * motorVoltage));

    }
    
}