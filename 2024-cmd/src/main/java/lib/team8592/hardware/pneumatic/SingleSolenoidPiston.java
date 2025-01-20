package lib.team8592.hardware.pneumatic;

import edu.wpi.first.wpilibj.*;

public class SingleSolenoidPiston {
    private Solenoid solenoid;

    public SingleSolenoidPiston(PneumaticsModuleType type, int channel) {
        solenoid = new Solenoid(type, channel);
    }

    /**
     * Applies power in the forward direction
     */
    public void extend() {
        this.setValue(true);
    }

    /**
     * Releases pressure from the solenoid and returns to natural state
     */
    public void retract() {
        this.setValue(false);
    }

    /**
     * Sets the direction that the solenoid extend
     * @param value the direction to extend towards
     */
    private void setValue(boolean extend) {
        solenoid.set(extend);
    }
}