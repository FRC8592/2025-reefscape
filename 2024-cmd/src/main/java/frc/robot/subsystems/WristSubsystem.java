package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Ports;
import lib.team8592.MatchMode;
import lib.team8592.PIDProfile;
import lib.team8592.hardware.motor.talonfx.KrakenX60Motor;

public class WristSubsystem extends NewtonSubsystem {
    private KrakenX60Motor wristMotor;

    private double desiredWristAngleDegrees = 0d;

    private PIDProfile wristGains = new PIDProfile();

    protected WristSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);
        this.wristMotor = new KrakenX60Motor(Ports.WRIST_MOTOR_CAN_ID);
        this.wristMotor.withGains(wristGains);
    }

    public void setWristAngle(double angle) {
        this.desiredWristAngleDegrees = angle;

        this.wristMotor.setPositionSmartMotion(angle / 1d);
    }

    private double fromWristDegreesToMotorRotations(double degrees) {
        return (degrees/360d) * Constants.WRIST.OVERALL_GEAR_RATIO;
    }

    private double fromMotorRotationsToWristDegrees(double rotations) {
        return (rotations*360d) / Constants.WRIST.OVERALL_GEAR_RATIO;
    }

    public double getWristAngleDegrees() {
        return this.fromMotorRotationsToWristDegrees(this.wristMotor.getRotations());
    }

    public boolean atTargetWristAngle() {
        return Math.abs(this.getWristAngleDegrees() - this.desiredWristAngleDegrees) < Constants.WRIST.WITHIN_THRESHOLD_DEGREES;
    }

    public Command setWristAngleCommand(double degrees) {
        return this.run(() -> {
            this.setWristAngle(degrees);
        }).until(() -> atTargetWristAngle());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Wrist Angle Degrees", desiredWristAngleDegrees);
        this.logger.log("Desired Motor Rotations", fromWristDegreesToMotorRotations(desiredWristAngleDegrees));
        this.logger.log("Current Wrist Angle Degrees", getWristAngleDegrees());
    }

    @Override
    public void stop() {
        this.wristMotor.setVelocity(desiredWristAngleDegrees);
    }
}