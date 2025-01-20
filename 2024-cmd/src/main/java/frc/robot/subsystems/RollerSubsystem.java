package frc.robot.subsystems;

import frc.robot.Ports;
import lib.team8592.MatchMode;
import lib.team8592.hardware.motor.spark.SparkFlexMotor;

public class RollerSubsystem extends NewtonSubsystem {
    private SparkFlexMotor topMotor;
    private SparkFlexMotor bottomMotor;

    private double targetTopMotorRPM = 0;
    private double targetBottomMotorRPM = 0;

    protected RollerSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.topMotor = new SparkFlexMotor(Ports.ROLLER_TOP_MOTOR_CAN_ID);
        this.bottomMotor = new SparkFlexMotor(Ports.ROLLER_BOTTOM_MOTOR_CAN_ID);
    }

    // Runs motors at specific velocities
    public void setTopMotorVelocity(double targetRPM) {
        this.topMotor.setPercentOutput(targetRPM / 6000.0);
        this.targetTopMotorRPM = targetRPM;
    }

    public void setBottomMotorVelocity(double targetRPM) {
        this.bottomMotor.setPercentOutput(targetRPM / 6000.0);
        this.targetBottomMotorRPM = targetRPM;
    }

    public void setTopMotorPercentOutput(double percentOutput) {
        this.topMotor.setPercentOutput(percentOutput);
        this.targetTopMotorRPM = percentOutput * this.topMotor.getMaxFreeVelocity();
    }

    public void setBottomMotorPercentOutput(double percentOutput) {
        this.bottomMotor.setPercentOutput(percentOutput);
        this.targetBottomMotorRPM = percentOutput * this.bottomMotor.getMaxFreeVelocity();
    }

    public double getTopMotorVelocity() {
        return topMotor.getVelocityRPM();
    }

    public double getBottomMotorVelocity() {
        return bottomMotor.getVelocityRPM();
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Top Motor Target RPM", targetTopMotorRPM);
        this.logger.log("Bottom Motor Target RPM", targetBottomMotorRPM);
        this.logger.log("Top Motor Current RPM", getTopMotorVelocity());
        this.logger.log("Bottom Motor Current RPM", getBottomMotorVelocity());
    }

    @Override
    public void stop() {
        setTopMotorVelocity(0d);
        setBottomMotorVelocity(0d);
    }
}
