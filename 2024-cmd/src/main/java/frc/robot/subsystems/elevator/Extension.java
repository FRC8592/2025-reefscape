package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.*;

public class Extension{
    private TalonFX leftExtensionMotor;
    private TalonFX rightExtensionMotor;
    private double targetExtension;

    public Extension(){
        leftExtensionMotor = new TalonFX(CAN.LEFT_EXTENSION_MOTOR_CAN_ID);
        rightExtensionMotor = new TalonFX(CAN.RIGHT_EXTENSION_MOTOR_CAN_ID);
        targetExtension = 0;
    }

    public double getExtensionPosition(){
        return leftExtensionMotor.getRot
    }

}
