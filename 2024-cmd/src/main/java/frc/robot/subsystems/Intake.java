package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Intake extends SubsystemBase {

    // private SparkFlexControl wristMotor = new SparkFlexControl(CAN.INTAKE_WRIST_MOTOR_CAN_ID, false);
    // private SparkFlexControl outerMotor = new SparkFlexControl(CAN.INTAKE_OUTER_MOTOR_CAN_ID,false);
    private SparkFlexControl innerMotor = new SparkFlexControl(CAN.INTAKE_INNER_MOTOR_CAN_ID,false);

    private double innerMotorCommandedVelocity = 0;
    // private double outerMotorCommandedVelocity = 0;
    // private double wristMotorCommandedVelocity = 0;


    // private DigitalInput beamBreak;

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
        // beamBreak = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
    }
    // Method that checks if the beam break is tripped 
    // public boolean isBeamBreakTripped() {
    //     return !beamBreak.get();
    // }

    // Runs motors at specific velocities
    public void runInnerMotor(double velocity) {
        innerMotor.setPercentOutput(velocity/6000.0);
        innerMotorCommandedVelocity = velocity;
    }

    // public void runOuterMotor(double velocity) {
    //     outerMotor.setVelocity(velocity);
    //     outerMotorCommandedVelocity = velocity;

    // }

    // public void runWristMotor(double velocity) {
    //     wristMotor.setVelocity(velocity);
    //     wristMotorCommandedVelocity = velocity;

    // }
    // stop all motors relating to intake
    public void stop() {
        // runOuterMotor(0);
        runInnerMotor(0);
        // runWristMotor(0);
    }

    public void periodic() {

        // Log velocities to smart dashboard
        // SmartDashboard.putBoolean("Beam Break Tripped", isBeamBreakTripped());

        // SmartDashboard.putNumber("Outer Motor Intake Velocity", getOuterMotorVelocity());
        SmartDashboard.putNumber("Inner Motor Intake Velocity", getInnerMotorVelocity());
        // SmartDashboard.putNumber("Wrist Motor Intake Velocity", getWristMotorVelocity());

        /* Log recorded velocities, commanded velocities, and difference between 
            commanded and recorded velocity for the outer intake motor, inner intake motor, 
            and wrist motor  
        */
        // Logger.recordOutput(INTAKE.LOG_PATH+"OuterMotorVelocity", getOuterMotorVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"InnerMotorVelocity", getInnerMotorVelocity());
        // Logger.recordOutput(INTAKE.LOG_PATH+"WristMotorVelocity", getWristMotorVelocity());

        // Logger.recordOutput(INTAKE.LOG_PATH+"OuterMotorCommandedVelocity", outerMotorCommandedVelocity);
        Logger.recordOutput(INTAKE.LOG_PATH+"InnerMotorCommandedVelocity", innerMotorCommandedVelocity);
        // Logger.recordOutput(INTAKE.LOG_PATH+"WristMotorCommandedVelocity", wristMotorCommandedVelocity);
        
        // Logger.recordOutput(INTAKE.LOG_PATH+"OuterCommanded - RecordedVelocity", outerMotorCommandedVelocity-getOuterMotorVelocity());
        Logger.recordOutput(INTAKE.LOG_PATH+"InnerCommanded - RecordedVelocity", innerMotorCommandedVelocity-getInnerMotorVelocity());
        // Logger.recordOutput(INTAKE.LOG_PATH+"WristCommanded - RecordedVelocity", wristMotorCommandedVelocity-getWristMotorVelocity());
        
    }

    // Methods that return motor velocities
    // public double getOuterMotorVelocity()
    // {
    //     return outerMotor.getVelocity();
    // }

    public double getInnerMotorVelocity()
    {
        return innerMotor.getVelocity();
    }

    // public double getWristMotorVelocity()
    // {
    //     return wristMotor.getVelocity();
    // }
}
