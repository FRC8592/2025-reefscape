package lib.team8592.hardware.motor.talonfx;

import lib.team8592.hardware.motor.MotorConstants;

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