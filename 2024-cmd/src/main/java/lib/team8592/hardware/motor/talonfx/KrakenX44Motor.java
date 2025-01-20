package lib.team8592.hardware.motor.talonfx;

import lib.team8592.hardware.motor.MotorConstants;

public class KrakenX44Motor extends TalonFXMotor {
    public KrakenX44Motor(int motorID) {
        this(motorID, false);
    }

    public KrakenX44Motor(int motorID, boolean inverted) {
        super(motorID, inverted, new MotorConstants(
            7530d, 
            275d, 
            4.05,
            630.7
        ));
    }
}