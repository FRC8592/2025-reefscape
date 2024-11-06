package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemCommands;

public class PivotCommands extends SubsystemCommands{
    private Pivot pivot;

    public PivotCommands(Pivot pivot){
        this.pivot = pivot;
    }

    public Command dropPivotCommand(){
        return pivot.run(() -> {
            pivot.dropPivot();
        });
    }

    public Command raisePivotCommand(){
        return pivot.run(() ->{
            pivot.raisePivot();
        });
    }
    @Override
    public Command stopCommand() {
        return pivot.run(() ->{
            pivot.stop();
        });
    }
    
}
