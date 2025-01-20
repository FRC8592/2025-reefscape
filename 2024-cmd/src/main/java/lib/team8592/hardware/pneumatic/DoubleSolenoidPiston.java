package lib.team8592.hardware.pneumatic;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidPiston {
    private DoubleSolenoid solenoid;

    public DoubleSolenoidPiston(PneumaticsModuleType type, int fwdChannel, int revChannel) {
        solenoid = new DoubleSolenoid(type, fwdChannel, revChannel);
    }

    /**
     * Applies power in the forward direction
     */
    public void extendForward() {
        this.setValue(Value.kForward);
    }

    /**
     * Applies power in the reverse direction
     */
    public void extendReverse() {
        this.setValue(Value.kReverse);
    }

    /**
     * Releases pressure from the solenoid and returns to natural state
     */
    public void turnOff() {
        this.setValue(Value.kOff);
    }

    /**
     * The direction that is currently applied
     * @return the {@code Value} that the piston is currently extended towards
     */
    public Value get() {
        return this.solenoid.get();
    }

    /**
     * Sets the direction that the solenoid extend
     * @param value the direction to extend towards
     */
    private void setValue(Value value) {
        solenoid.set(value);
    }
}