package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

public final class NewtonCommands {
    private static Swerve swerve;
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

    //  // command for taking in coral 
    //  public static Command intakeCommand(){
    //     return intake.run(() ->{
    //         intake.runInnerMotor(INTAKE.INNER_MOTOR_INTAKE_VELOCITY);
    //     });

    // }

    // //command for the release of the coral for scoring 
    // public static Command outakeCommand(){
    //     return intake.run(() ->{
    //         intake.runInnerMotor(INTAKE.INNER_MOTOR_OUTAKE_VELOCITY);
    //     });
    // }

    // public static Command stopIntakeComand(){
    //     return intake.run(() -> {
    //         intake.runInnerMotor(0);
    //     });
    // }
}
