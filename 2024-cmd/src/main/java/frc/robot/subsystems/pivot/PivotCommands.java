package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.subsystems.pivot.Pivot.Positions;

public class PivotCommands extends SubsystemCommands {
    private Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }

    public Command setPivotPositionCommand(Positions position){
        return Commands.none();
    }

    public Command stopCommand(){
        return Commands.none();
    }
}
