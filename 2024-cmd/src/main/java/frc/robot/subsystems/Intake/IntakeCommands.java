package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.SubsystemCommands;

public class IntakeCommands extends SubsystemCommands {
    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;;
    }

    public Command runIntakeCommand() {
        return intake.runEnd(() -> {
            intake.setIntakeVelocity(INTAKE.INTAKE_VELOCITY);
        },() -> {
            intake.stop();
        });
    }

    public Command runOuttakeCommand(){
        return intake.runEnd(() -> {
            intake.setIntakeVelocity(INTAKE.OUTAKE_VELOCITY);
        },() -> {
            intake.stop();
        });
    }

    @Override
    public Command stopCommand() {
        return intake.run(() -> {
            intake.stop();
        });
    }
}