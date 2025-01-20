package frc.robot.subsystems.swerve.ctreswerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CTRESwerve {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldRelative = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotRelative = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    public void drive(ChassisSpeeds speeds, boolean driveRobotRelative) {
        if (driveRobotRelative) {
            drivetrain.setControl(
                robotRelative.withVelocityX(speeds.vxMetersPerSecond) 
                    .withVelocityY(speeds.vyMetersPerSecond) 
                    .withRotationalRate(speeds.omegaRadiansPerSecond) 
            );

            this.targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        else {
        drivetrain.setControl(
            fieldRelative.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
                .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
                .withRotationalRate(speeds.omegaRadiansPerSecond) // Drive counterclockwise with negative X (left)
            );

            this.targetSpeeds = speeds;
        }
    }

    public ChassisSpeeds getCommandedSpeeds() {
        return this.targetSpeeds;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public void resetHeading() {
        drivetrain.seedFieldCentric();
    }

    public Rotation2d getYaw() {
        return drivetrain.getState().Pose.getRotation();
    };

    public Pose2d getCurrentOdometryPosition() {
        return drivetrain.getState().Pose;
    }

    public void setKnownOdometryPose(Pose2d currentPose) {        
        drivetrain.resetPose(currentPose);
    }

    public void setBraking() {
        drivetrain.setControl(brake);
    }

    public void pointWheelsAt(Rotation2d angle) {
        drivetrain.setControl(point.withModuleDirection(angle));
    }

    public void periodic(){
        drivetrain.periodic();
    }
}
