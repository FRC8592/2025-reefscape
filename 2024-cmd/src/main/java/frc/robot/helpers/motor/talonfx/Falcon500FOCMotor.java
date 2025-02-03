package frc.robot.helpers.motor.talonfx;

import frc.robot.helpers.motor.MotorConstants;

public class Falcon500FOCMotor extends TalonFXMotor {
    public Falcon500FOCMotor(int motorID) {
        this(motorID, false);
    }

    public Falcon500FOCMotor(int motorID, boolean inverted) {
        super(motorID, inverted, new MotorConstants(
            6080d, 
            304d, 
            5.84,
            509.2
        ));
    }
}