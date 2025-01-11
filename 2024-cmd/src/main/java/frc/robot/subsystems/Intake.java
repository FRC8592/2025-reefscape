package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Intake extends SubsystemBase {

    private SparkFlexControl wristMotor = new SparkFlexControl(CAN.INTAKE_WRIST_MOTOR_CAN_ID, false);
    private SparkFlexControl outerMotor = new SparkFlexControl(CAN.INTAKE_OUTER_MOTOR_CAN_ID,false);
    private SparkFlexControl innerMotor = new SparkFlexControl(CAN.INTAKE_INNER_MOTOR_CAN_ID,false);

    private double innerMotorCommandedVelocity = 0;
    private double outerMotorCommandedVelocity = 0;
    private double wristMotorCommandedVelocity = 0;


    private DigitalInput beamBreak;

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
        beamBreak = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
    }
  
    public boolean isBeamBreakTripped() {
        return !beamBreak.get();
    }

    public void runInnerMotor(double velocity) {
        innerMotor.setVelocity(velocity);
        innerMotorCommandedVelocity = velocity;
    }

    public void runOuterMotor(double velocity) {
        outerMotor.setVelocity(velocity);
        outerMotorCommandedVelocity = velocity;

    }

    public void runWristMotor(double velocity) {
        wristMotor.setVelocity(velocity);
        wristMotorCommandedVelocity = velocity;

    }

    public void stop() {
        runOuterMotor(0);
        runInnerMotor(0);
        runWristMotor(0);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Beam Break Tripped", isBeamBreakTripped());

        SmartDashboard.putNumber("Outer Motor Intake Velocity", getOuterMotorVelocity());
        SmartDashboard.putNumber("Inner Motor Intake Velocity", getInnerMotorVelocity());
        SmartDashboard.putNumber("Wrist Motor Intake Velocity", getWristMotorVelocity());

        Logger.recordOutput(INTAKE.LOG_PATH+"OuterMotorVelocity", getOuterMotorVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"InnerMotorVelocity", getInnerMotorVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"WristMotorVelocity", getWristMotorVelocity());

        Logger.recordOutput(INTAKE.LOG_PATH+"OuterMotorCommandedVelocity", outerMotorCommandedVelocity);
        Logger.recordOutput(INTAKE.LOG_PATH+"InnerMotorCommandedVelocity", innerMotorCommandedVelocity);
        Logger.recordOutput(INTAKE.LOG_PATH+"WristMotorCommandedVelocity", wristMotorCommandedVelocity);
    }

    public double getOuterMotorVelocity()
    {
        return outerMotor.getVelocity();
    }

    public double getInnerMotorVelocity()
    {
        return innerMotor.getVelocity();
    }

    public double getWristMotorVelocity()
    {
        return wristMotor.getVelocity();
    }
}
