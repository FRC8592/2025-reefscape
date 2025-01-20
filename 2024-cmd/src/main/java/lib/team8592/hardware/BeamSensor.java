package lib.team8592.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamSensor {
    private DigitalInput sensor; // Any digital input beam sensor

    public BeamSensor(int port) {
        this.sensor = new DigitalInput(port);
    }

    /**
     * Whether the beam has been broken on the sensor; flipped because false means tripped on the sensor
     */
    public boolean isTripped() {
        return !this.sensor.get();
    }
}