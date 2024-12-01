package frc.robot;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

public class CarrotDrivetrain {
    private TalonFX testMotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);
    VoltageOut testMotorVoltage = new VoltageOut(0);
    public CarrotDrivetrain () {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        //set the factory defaults for all motors
        testMotor.getConfigurator().apply(talonFXConfigs);

        //set motor to 0 power for saftery reasons
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
