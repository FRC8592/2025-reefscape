package frc.robot;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;



public class TestMotors{
    private TalonFX testMotor;
    private VoltageOut testMotorVoltage = new  VoltageOut(0); //Datatype needed to contorl voltage 

    public TestMotors(){
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

        //Create the contollaer object for our test motor 
        testMotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);

        //Set the factory defaults for all motor configuration options 
        testMotor.getConfigurator().apply(talonFXConfig);
        
        // Set the motor to 0 for safety 
        testMotor.setControl(testMotorVoltage.withOutput(0));
    } 


    /**
     * Set motor power in terms of percent
     * @param outputPercent Specify motor power in a range of -1 to +1
     */
    public void setPercentMotorOutput(double outputPercent) {
    double motorVoltage = testMotor.getSupplyVoltage().getValueAsDouble();

    testMotor.setControl(testMotorVoltage.withOutput(motorVoltage*outputPercent));

    }
}