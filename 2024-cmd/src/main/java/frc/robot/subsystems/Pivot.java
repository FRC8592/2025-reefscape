package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Pivot extends SubsystemBase{
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
}
