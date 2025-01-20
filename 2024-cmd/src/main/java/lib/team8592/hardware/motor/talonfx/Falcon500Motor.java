package lib.team8592.hardware.motor.talonfx;

import lib.team8592.hardware.motor.MotorConstants;

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