package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public final class NewtonCommands {
    private static Swerve swerve;
    private static Elevator elevator;
    public static void addSubsystems(Swerve swerve){
        NewtonCommands.swerve = swerve;
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

    public static Command primeL1Command(){
        return Commands.none();
    }

    public static Command primeL2Command(){
        return Commands.none();
    }

    public static Command primeL3Command(){
        return Commands.none();
    }

    public static Command primeL4Command(){
        return Commands.none();
    }

    public static Command humanPlayerIntake(){
        return Commands.none();
    }

    public static Command stowCommand(){
        return Commands.none();
    }

    public static Command removeAlgaeCommand(){
        return Commands.none();
    }


    /**
     * Currently Commands.none(). Update this comment when the command is added.
     *
     * @param position
     * @return Commands.none()
     */

    /**
     * Command to stop the intake and stow the pivot to REST position
     * @return the command
     */
}
