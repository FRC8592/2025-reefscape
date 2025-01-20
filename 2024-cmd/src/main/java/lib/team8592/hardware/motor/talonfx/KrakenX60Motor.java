package lib.team8592.hardware.motor.talonfx;

import lib.team8592.hardware.motor.MotorConstants;

public class KrakenX60Motor extends TalonFXMotor {
    public KrakenX60Motor(int motorID) {
        this(motorID, false);
    }

    public KrakenX60Motor(int motorID, boolean inverted) {
        super(motorID, inverted, new MotorConstants(
            6000d, 
            366d, 
            7.09,
            502.1
        ));
    }
}