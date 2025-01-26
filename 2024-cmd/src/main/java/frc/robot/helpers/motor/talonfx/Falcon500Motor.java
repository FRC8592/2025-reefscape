package frc.robot.helpers.motor.talonfx;

import frc.robot.helpers.motor.MotorConstants;

public class Falcon500Motor extends TalonFXMotor {
    public Falcon500Motor(int motorID) {
        this(motorID, false);
    }

    public Falcon500Motor(int motorID, boolean inverted) {
        super(motorID, inverted, new MotorConstants(
            6380d, 
            257d, 
            4.69,
            534.8
        ));
    }
}