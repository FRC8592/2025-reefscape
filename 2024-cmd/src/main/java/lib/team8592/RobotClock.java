package lib.team8592;

import edu.wpi.first.wpilibj.Timer;

public class RobotClock extends Timer {
    private double lastTime = 0.0;
    private double curTime = 0.0;

    public double dt() {
        return curTime - lastTime;
    }

    public void update() {
        super.start();
        lastTime = curTime;
        curTime = get();
    }
}