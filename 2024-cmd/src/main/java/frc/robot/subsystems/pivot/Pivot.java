package frc.robot.subsystems.pivot;

import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.Constants.*;

public class Pivot extends NewtonSubsystem{
    private static Pivot instance = null;
    public static Pivot getInstance(){
        if(instance == null){
            throw new IllegalStateException("The Pivot subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static Pivot instantiate(){
        if(instance != null){
            throw new IllegalStateException("The Pivot subsystem can't be instantiated twice");
        }
        instance = new Pivot();
        return instance;
    }

    public PivotCommands commands = new PivotCommands(this);

    public enum Positions {
        GROUND(PIVOT.GROUND_DEGREES),
        REST(PIVOT.REST_DEGREES),
        HP_LOAD(PIVOT.HP_LOAD_DEGREES),
        SCORE_HIGH(PIVOT.SCORE_HIGH_DEGREES),;

        public int degrees = 0;
        Positions (int degrees){
            this.degrees = degrees;
        }
    }

    public void stop(){
    }
}
