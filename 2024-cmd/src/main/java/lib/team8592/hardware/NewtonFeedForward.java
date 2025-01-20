package lib.team8592.hardware;

import java.util.function.DoubleSupplier;

public class NewtonFeedForward {
    public double kV = 0d;
    public double kA = 0d;
    public double kS = 0d;
    public double kG = 0d;

    private DoubleSupplier angle = () -> 0d;

    private double lastVelocity = 0d;

    public NewtonFeedForward() {}

    public NewtonFeedForward(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public NewtonFeedForward setV(double gain) {
        this.kV = gain;
        return this;
    }

    public NewtonFeedForward setA(double gain) {
        this.kA = gain;
        return this;
    }

    public NewtonFeedForward setS(double gain) {
        this.kS = gain;
        return this;
    }

    public NewtonFeedForward withKG(double kG, DoubleSupplier angle) {
        this.kG = kG;
        this.angle = angle;
        return this;
    }

    private double get(double velocity, double acceleration) {
        this.lastVelocity = velocity;

        return this.kG * Math.cos(this.angle.getAsDouble()) + this.kS + this.kV * velocity + this.kA * acceleration;
    }

    public double calculate(double velocity, double dt) {
        return this.get(velocity, (velocity - lastVelocity) / dt);
    }
}