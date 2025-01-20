package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Ports;
import lib.team8592.*;
import lib.team8592.hardware.motor.talonfx.KrakenX60Motor;

public class ClockSubsystem extends NewtonSubsystem {
    private KrakenX60Motor clockMotor;

    private double desiredClockAngleDegrees = 0d;

    private PIDProfile clockGains = new PIDProfile();

    protected ClockSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);
        this.clockMotor = new KrakenX60Motor(Ports.CLOCK_MOTOR_CAN_ID);
        this.clockMotor.withGains(clockGains);
    }

    public void setClockAngle(double angle) {
        this.desiredClockAngleDegrees = angle;

        this.clockMotor.setPositionSmartMotion(angle / 1d);
    }

    private double fromClockDegreesToMotorRotations(double degrees) {
        return (degrees/360d) * Constants.CLOCK.OVERALL_GEAR_RATIO;
    }

    private double fromMotorRotationsToClockDegrees(double rotations) {
        return (rotations*360d) / Constants.CLOCK.OVERALL_GEAR_RATIO;
    }

    public double getClockAngleDegrees() {
        return this.fromMotorRotationsToClockDegrees(this.clockMotor.getRotations());
    }

    public boolean atTargetClockAngle() {
        return Math.abs(this.getClockAngleDegrees() - this.desiredClockAngleDegrees) < Constants.CLOCK.WITHIN_THRESHOLD_DEGREES;
    }

    public Command setClockAngleCommand(double degrees) {
        return this.run(() -> {
            this.setClockAngle(degrees);
        }).until(() -> atTargetClockAngle());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Clock Angle Degrees", desiredClockAngleDegrees);
        this.logger.log("Desired Motor Rotations", fromClockDegreesToMotorRotations(desiredClockAngleDegrees));
        this.logger.log("Current Clock Angle Degrees", getClockAngleDegrees());
    }

    @Override
    public void stop() {
        this.clockMotor.setVelocity(desiredClockAngleDegrees);
    }
}