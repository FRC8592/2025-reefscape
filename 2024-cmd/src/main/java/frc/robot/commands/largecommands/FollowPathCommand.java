package frc.robot.commands.largecommands;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class FollowPathCommand extends LargeCommand{
    // Pathing variables
    private Trajectory trajectory;
    private Timer timer = new Timer();

    // Alternate movement variables
    private BooleanSupplier useAlternateRotation = () -> false;
    private Supplier<Rotation2d> alternateRotation = () -> new Rotation2d();
    private BooleanSupplier useAlternateTranslation = () -> false;
    private Supplier<ChassisSpeeds> alternateTranslation = () -> new ChassisSpeeds();

    // PID controllers
    private ProfiledPIDController turnController;
    private HolonomicDriveController drivePID;
    private PIDController xController;
    private PIDController yController;

    // Whether to flip to the red side of the field
    private BooleanSupplier flip;

    /**
     * Command to follow a trajectory
     *
     * @param trajectory the trajectory to follow
     * @param flip lambda that returns whether to mirror the path to the
     * red side of the field.
     */
    public FollowPathCommand(Trajectory trajectory, BooleanSupplier flip){
        super(swerve);

        this.trajectory = trajectory;

        this.xController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );
        this.xController.setTolerance(
            SWERVE.PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE
        );

        this.yController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );
        this.yController.setTolerance(
            SWERVE.PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE
        );

        this.turnController = new ProfiledPIDController(
            SWERVE.PATH_FOLLOW_ROTATE_kP,
            SWERVE.PATH_FOLLOW_ROTATE_kI,
            SWERVE.PATH_FOLLOW_ROTATE_kD,
            new Constraints(
                SWERVE.PATH_FOLLOW_ROTATE_MAX_VELOCITY,
                SWERVE.PATH_FOLLOW_ROTATE_MAX_ACCELLERATION
            )
        );
        this.turnController.setTolerance(
            SWERVE.PATH_FOLLOW_ROTATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_ROTATE_VELOCITY_TOLERANCE
        );
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);

        this.drivePID = new HolonomicDriveController(xController, yController, turnController);
        this.drivePID.setTolerance(
            new Pose2d(
                new Translation2d(xController.getPositionTolerance(), yController.getPositionTolerance()),
                new Rotation2d(turnController.getPositionTolerance())
            )
        );

        this.flip = flip;
    }

    /**
     * Command to follow a path with the option to deviate from it as configured by the parameters
     *
     * @param trajectory the path to follow
     *
     * @param flip lambda that returns whether the path should be mirrored to the red side of the
     * field.
     *
     * @param useAlternateRotation a lambda that returns whether to use the alternate rotation
     * provided by {@code rotationSupplier}
     *
     * @param rotationSupplier a lambda that returns the alternate rotation to be used when
     * {@code useAlternateRotation} returns {@code true}. NOTE: The degree and radian values stored
     * in the Rotation2d are used as a velocity setpoint, not a position setpoint, so you have to run
     * your own control loop (as needed) for this to work.
     *
     * @param useAlternateTranslation a lambda that returns whether to use the alternate translation
     * provided by {@code translationSupplier}
     *
     * @param translationSupplier a lambda that returns the alternate translation to be used when
     * {@code useAlternatTranslation} returns {@code true}. The rotation component of the
     * {@code ChassisSpeeds} is ignored.
     */
    public FollowPathCommand(
        Trajectory trajectory, BooleanSupplier flip,
        BooleanSupplier useAlternateRotation, Supplier<Rotation2d> rotationSupplier,
        BooleanSupplier useAlternateTranslation, Supplier<ChassisSpeeds> translationSupplier
    ){
        this(trajectory, flip);
        this.useAlternateRotation = useAlternateRotation;
        this.alternateRotation = rotationSupplier;
        this.useAlternateRotation = useAlternateTranslation;
        this.alternateTranslation = translationSupplier;
    }

    public void initialize(){
        timer.reset();
        timer.start();

        // Stop the swerve
        swerve.drive(new ChassisSpeeds());
    }
    public void execute(){
        // Instances of State contain information about pose, velocity, accelleration, curvature, etc.
        State desiredState = trajectory.sample(timer.get());

        if(flip.getAsBoolean()){
            desiredState = flip(desiredState);
        }

        if(Robot.isReal()){
            Logger.recordOutput(SWERVE.LOG_PATH+"TargetPose", desiredState.poseMeters);

            ChassisSpeeds driveSpeeds = drivePID.calculate(
                swerve.getCurrentPosition(),
                desiredState,
                desiredState.poseMeters.getRotation()
            );

            // Override the rotation speed (NOT position target) if useAlternativeRotation
            // returns true.
            if(useAlternateRotation.getAsBoolean()){
                driveSpeeds.omegaRadiansPerSecond = alternateRotation.get().getRadians();
            }

            // Same, except overriding translation only
            if(useAlternateTranslation.getAsBoolean()){
                driveSpeeds.vxMetersPerSecond = alternateTranslation.get().vxMetersPerSecond;
                driveSpeeds.vyMetersPerSecond = alternateTranslation.get().vyMetersPerSecond;
            }

            swerve.drive(driveSpeeds);
        }
        else{
            // The swerve automatically sets the visible position on the simulated
            // field, so all we have to do is set its known position
            swerve.resetPose(desiredState.poseMeters);
        }
    }
    public void end(boolean interrupted){
        swerve.drive(new ChassisSpeeds());
    }
    public boolean isFinished(){
        return ( // Only return true if enough time has elapsed, we're at the target location, and we're not using alternate movement.
            timer.hasElapsed(trajectory.getTotalTimeSeconds())
            && (drivePID.atReference() || !Robot.isReal())
            && !useAlternateRotation.getAsBoolean()
            && !useAlternateTranslation.getAsBoolean()
        );
    }

    /**
     * Mirror a State object to the other side of the field.
     *
     * @param state {@code State}: the state to mirror
     * @return the mirrored state
     */
    private State flip(State state){
        return new State(
            state.timeSeconds,
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            new Pose2d(
                MEASUREMENTS.FIELD_LENGTH_METERS - state.poseMeters.getX(),
                state.poseMeters.getY(),
                Rotation2d.fromRadians(Math.PI).minus(state.poseMeters.getRotation())
            ),
            -state.curvatureRadPerMeter
        );
    }
}