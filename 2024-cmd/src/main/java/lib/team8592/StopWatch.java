package lib.team8592;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.Timer;

public class StopWatch implements Sendable {
    private Timer timer = new Timer();
    private double stopTime = 0d;
    private boolean hasStarted = false;

    public StopWatch(double stopTime) {
        this.stopTime = stopTime;
        this.timer.reset();

        SendableRegistry.addLW(this, "StopWatch");
    }

    public void start() {
        if (!hasStarted) {
            this.timer.start();
            hasStarted = true;
        }
    }

    public void stop() {
        this.timer.stop();
        this.timer.reset();
    }

    public double getElapsed() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.get();
    }

    public double getRemaining() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.get() - stopTime;
    }

    public boolean hasFinished() {
        if (!hasStarted) {
            this.start();
            this.hasStarted = true;
        }
        return this.timer.hasElapsed(stopTime);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("StopWatch");
        builder.addDoubleProperty("Stop Time", 
            () -> stopTime, 
            (time) -> stopTime = time
        );
    }
}