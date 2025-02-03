package frc.robot.helpers.motor;

public class MotorConstants {
    public final double MAX_VELOCITY_RPM;
    public final double STALL_CURRENT_AMPS;
    public final double STALL_TORQUE_NM;
    public final double MOTOR_KV;

    public MotorConstants(double maxRPM, double stallAmp, double stallTorque, double motorKV) {
        this.MAX_VELOCITY_RPM = maxRPM;
        this.STALL_CURRENT_AMPS = stallAmp;
        this.STALL_TORQUE_NM = stallTorque;
        this.MOTOR_KV = motorKV;
    }
}