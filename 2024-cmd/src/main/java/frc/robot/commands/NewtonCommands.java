package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public final class NewtonCommands {
    private static Swerve swerve;
    private static Intake intake;
    public static void addSubsystems(Swerve swerve,Intake intake){
        NewtonCommands.swerve = swerve;
        NewtonCommands.intake = intake;

    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param driveX a lambda returning the driver's X input
     * @param driveY a lambda returning the driver's Y input
     *
     * @return the command
     */
    public static Command swerveSnapToCommand(Rotation2d angle, DoubleSupplier driveX, DoubleSupplier driveY){
        return swerve.run(() -> {
            ChassisSpeeds processed = swerve.processJoystickInputs(
                driveX.getAsDouble(),
                driveY.getAsDouble(),
                0
            );
            processed.omegaRadiansPerSecond = swerve.snapToAngle(angle);
            swerve.drive(processed, DriveModes.AUTOMATIC);
        });
    }
    // intake commands for intake, outake, and 
    public static Command intakeCommand(){
        return intake.run(()-> {
            intake.runInnerMotor(INTAKE.INNER_MOTOR_INTAKE_VELOCITY);
            intake.runOuterMotor(INTAKE.OUTER_MOTOR_INTAKE_VELOCITY);
        }).until(()-> {
            return intake.isBeamBreakTripped();
        });
    }

    public static Command outtakeCommand() {
        return intake.run(() -> {
            intake.runInnerMotor(INTAKE.INNER_MOTOR_OUTAKE_VELOCITY);
            intake.runOuterMotor(INTAKE.OUTER_MOTOR_OUTAKE_VELOCITY);
        }).until(() -> {
            return !intake.isBeamBreakTripped();
        });
    }
}


