package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.proxies.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.DriveModes;

public final class NewtonCommands {
    private static SubsystemManager manager;
    public static void addSubsystems(SubsystemManager manager){
        NewtonCommands.manager = manager;
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
        return manager.swerve.run(() -> {
            ChassisSpeeds processed = manager.swerve.processJoystickInputs(
                driveX.getAsDouble(),
                driveY.getAsDouble(),
                0
            );
            processed.omegaRadiansPerSecond = manager.swerve.snapToAngle(angle);
            manager.swerve.drive(processed, DriveModes.AUTOMATIC);
        });
    }

    public static Command driveToReefCommand(double offset, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotate) {
        return manager.swerve.run(() -> {
            if (manager.vision.isAnyTargetVisible()) { // We see apriltag
                manager.swerve.drive(manager.vision.driveToReef(offset));
            } else {
                manager.swerve.drive(manager.swerve.processJoystickInputs(
                    driveX.getAsDouble(), 
                    driveY.getAsDouble(),
                    rotate.getAsDouble()
                ));
            }
        });
    }

    public static Command setRobotState(RobotStates state){
        double targetWrist = state.wristDegrees;
        double targetClock = state.clockDegrees;
        double targetElevator = state.elevatorInches;

        Command commandSequence = Commands.none();

        // Swing out the clock enough for us to start moving the wrist and elevator
        if (targetClock <= Constants.SUPERSTRUCTURE.CLOCK_SWING_OUT_ANGLE) { // Want to swing clock clockwise
            commandSequence = manager.clock.setClockAngleCommand(Constants.SUPERSTRUCTURE.CLOCK_SWING_OUT_ANGLE);
        } else {
            commandSequence = manager.clock.setClockAngleCommand(targetClock);
        }

        // Move the wrist and elevator to the target position if clock past threshold
        commandSequence = commandSequence.alongWith(
            new WaitUntilCommand(
                manager.wrist.setWristAngleCommand(targetWrist)
                .alongWith(manager.elevator.setElevatorHeightCommand(targetElevator)),
                () -> manager.clock.getClockAngleDegrees() >= Constants.SUPERSTRUCTURE.CLOCK_SWING_OUT_ANGLE
            )
        );

        // if (currentElevator <= Constants.SUPERSTRUCTURE.FUNNEL_TOP_FROM_BASE_ELEVATOR_INCHES) { // Elevator below funnel
        //     if (currentWrist <= Constants.SUPERSTRUCTURE.WRIST_LOCKED_UNDER_FUNNEL_THRESHOLD) {
        //         if (targetClock >= 45d) { // Target angle for the wrist is past our swing out threshold

        //         } else {

        //         }

        //         // Sequence ...
        //         //
        //         // 1) Swing clock arm out of the way
        //         // 2) Move wrist and elevator to target positions
        //         // 3) Move clock arm back to target position

        //         new SequentialCommandGroup(
        //             manager.clock.setClockAngleCommand(Constants.SUPERSTRUCTURE.CLOCK_SWING_OUT_ANGLE), // Swing clock arm out of the way first
        //             new ParallelCommandGroup(
        //                 manager.wrist.setWristAngleCommand(targetWrist), // Move wrist to target position
        //                 manager.elevator.setElevatorHeightCommand(targetElevator), // Raise elevator to target position
        //                 new WaitUntilCommand(
        //                     manager.clock.setClockAngleCommand(targetClock),
        //                     () -> currentElevator >= Constants.SUPERSTRUCTURE.FUNNEL_TOP_FROM_BASE_ELEVATOR_INCHES
        //                 )
        //             )
        //         );


        //         // If wrist is current under the funnel, we have to swing the clock arm out first to move the wrist
        //         // commandSequence = new SequentialCommandGroup(
        //         //     manager.clock.setClockAngleCommand(45d)) // Swing clock arm out of the way first
        //         //     .andThen(
        //         //         manager.wrist.setWristAngleCommand(targetWrist) // Move wrist to target position
        //         //         .alongWith(
        //         //             manager.elevator.setElevatorHeightCommand(targetElevator),
        //         //             new WaitUntilCommand(
        //         //                 manager.clock.setClockAngleCommand(targetClock),
        //         //                 () -> currentElevator >= Constants.SUPERSTRUCTURE.FUNNEL_TOP_FROM_BASE_ELEVATOR_INCHES
        //         //             )
        //         //         )
        //         //     );
        //     } else {
        //         // If the wrist is not under the funnel, we can move the wrist directly
        //         commandSequence = commandSequence
        //             .andThen(
        //                 manager.wrist.setWristAngleCommand(targetWrist)
        //                     .alongWith(
        //                         manager.elevator.setElevatorHeightCommand(targetElevator),
        //                         manager.clock.setClockAngleCommand(targetClock)
        //                     )
        //             );
        //     }
        // }

        return commandSequence;
    }

    // command for taking in coral
    public static Command intakeCommand(){
        return manager.rollers.run(()-> {
            manager.rollers.setTopMotorVelocity(INTAKE.INNER_MOTOR_INTAKE_VELOCITY);
        });
    }
    // command for release coral for scoring
    public static Command outtakeCommand() {
        return manager.rollers.run(() -> {
            manager.rollers.setTopMotorVelocity(INTAKE.INNER_MOTOR_OUTAKE_VELOCITY);
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

    public static Command groundIntakeCommand(){
        return Commands.none();
    }

    public static Command stowCommand(){
        return Commands.none();
    }

    public static Command primeL2AlgaeCommand(){
        return Commands.none();
    }

    public static Command primeL3AlgaeCommand(){
        return Commands.none();
    }

    public static Command goToPrimePositionCommand(){
        return Commands.none();
    }

    public static Command primeProcessorCommand(){
        return Commands.none();
    }

    public static Command primeNetCommand(){
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


