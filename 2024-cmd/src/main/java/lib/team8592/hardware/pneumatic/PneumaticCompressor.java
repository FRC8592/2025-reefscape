package lib.team8592.hardware.pneumatic;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticCompressor {
    private Compressor compressor;

    public PneumaticCompressor(PneumaticsModuleType type) {
        compressor = new Compressor(type);
    }

    /**
     * Turns the compressor on
     */
    public void start() {
        compressor.enableDigital();
    }

    /**
     * Turns the compressor on in analog mode ONLY IF SUPPORTED
     * @param minPressure The minimum ideal pressure
     * @param maxPressure The absolute maximum pressure
     */
    public void startAnalog(double minPressure, double maxPressure) {
        compressor.enableAnalog(minPressure, maxPressure);
    }

    /**
     * Turns the compressor off
     */
    public void stop() {
        compressor.disable();
    }
}