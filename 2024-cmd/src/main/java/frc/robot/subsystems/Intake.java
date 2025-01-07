package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Intake extends SubsystemBase {
    //private TalonFX topMotor = new TalonFX(CAN.INTAKE_TOP_MOTOR_CAN_ID);
    private SparkFlexControl topMotor = new SparkFlexControl(CAN.INTAKE_TOP_MOTOR_CAN_ID, false);
    // private TalonFX bottomMotor = new TalonFX(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID);
    private SparkFlexControl bottomMotor = new SparkFlexControl(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID, false);

    private DigitalInput beamBreak;

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
        beamBreak = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
        topMotor.setInverted();
    }
  
    public boolean isBeamBreakTripped() {
        return !beamBreak.get();
    }

    public void runTopMotor(double speedRPM) {
        topMotor.setPercentOutput(speedRPM);
        //topMotor.setControl(voltage);
    }

    public void runBottomMotor(double speedRPM) {
        bottomMotor.setPercentOutput(speedRPM);
    }

    public double getTopMotorVelocity() {
        return topMotor.getVelocity();
    }

    public double getBottomMotorVelocity() {
        return bottomMotor.getVelocity();
    }

    public void stop() {
        runTopMotor(0);
        runBottomMotor(0);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Beam Break Tripped", isBeamBreakTripped());

        SmartDashboard.putNumber("Top Motor Intake Velocity", getTopMotorVelocity());
        SmartDashboard.putNumber("Bottom Motor Intake Velocity", getBottomMotorVelocity());
    };
}
