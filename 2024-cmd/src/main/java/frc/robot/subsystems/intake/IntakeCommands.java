package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemCommands;

public class IntakeCommands extends SubsystemCommands {
    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;

    }

    public Command stopCommand(){
        return Commands.none();
    }
}
