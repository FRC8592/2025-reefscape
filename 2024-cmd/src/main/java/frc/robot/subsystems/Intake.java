package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Intake extends SubsystemBase {
    //private TalonFX topMotor = new TalonFX(CAN.INTAKE_TOP_MOTOR_CAN_ID);
    private SparkFlexControl wristMotor = new SparkFlexControl(CAN.INTAKE_WRIST_MOTOR_CAN_ID);
    // private TalonFX bottomMotor = new TalonFX(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID);
    private SparkFlexControl gripMotor = new SparkFlexControl();

    private DigitalInput beamBreak;

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
        beamBreak = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
        // topMotor.setInverted();
    }
  
//     public boolean isBeamBreakTripped() {
//     }

    // public void stop() {
    //     runTopMotor(0);
    //     runBottomMotor(0);
    // }

//     public void periodic() {
//         SmartDashboard.putBoolean("Beam Break Tripped", isBeamBreakTripped());

//         SmartDashboard.putNumber("Top Motor Intake Velocity", getTopMotorVelocity());
//         SmartDashboard.putNumber("Bottom Motor Intake Velocity", getBottomMotorVelocity());
//     };
}
