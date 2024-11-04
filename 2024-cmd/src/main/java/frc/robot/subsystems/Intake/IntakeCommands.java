package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemCommands;

public class IntakeCommands extends SubsystemCommands {
    private Intake intake;

    public IntakeCommands(Intake intakeBucket) {
        intake = intakeBucket;
    }

    public Command intakeCommand() {
        return intake.run(() -> {
            intake.setIntakeVelocity(Constants.INTAKE.INTAKE_VELOCITY);
        });
    }

    public Command outtakeCommand(){
        return intake.run(() -> {
            // Add outaking code here
            intake.setIntakeVelocity(Constants.INTAKE.OUTAKE_VELOCITY);
        });
    }

    @Override
    public Command stopCommand() {
        return intake.run(() -> {
            intake.stop();
        });
    }
}