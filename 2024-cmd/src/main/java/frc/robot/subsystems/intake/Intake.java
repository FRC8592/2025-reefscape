package frc.robot.subsystems.intake;

import frc.robot.subsystems.NewtonSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.SparkMaxControl;

public class Intake extends NewtonSubsystem {
    //private TalonFX topMotor = new TalonFX(CAN.INTAKE_TOP_MOTOR_CAN_ID);
    private SparkMaxControl topMotor = new SparkMaxControl(CAN.INTAKE_TOP_MOTOR_CAN_ID,true);
    // private TalonFX bottomMotor = new TalonFX(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID);
    private SparkFlexControl bottomMotor = new SparkFlexControl(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID, true);

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
    }

    private void runTopMotor(int speedRPM) {
        topMotor.setPercentOutput(speedRPM/6000d);
        //topMotor.setControl(voltage);
    }

    private void runBottomMotor(int speedRPM) {
        bottomMotor.setPercentOutput(speedRPM/6000d);
    }

    public void stop() {
    }

    /**
     * Command to intake the bucket
     * 
     * @return the command
     */
    public Command intakeCommand() {
        return this.run(() -> {
            this.runTopMotor(INTAKE.TOP_MOTOR_INTAKE_SPEED);
            this.runBottomMotor(INTAKE.BOTTOM_MOTOR_INTAKE_SPEED);
        });
    }

    /**
     * Command to put bucket into the low goal or the grid
     * 
     * @return the command
     */
    public Command scoreCommand() {
        return this.run(() -> {
            this.runTopMotor(INTAKE.TOP_MOTOR_SCORE_SPEED);
            this.runBottomMotor(INTAKE.BOTTOM_MOTOR_SCORE_SPEED);
        });
    }

    /**
     * Command to spit out the bucket
     * 
     * @return the command
     */
    public Command outtakeCommand() {
        return this.run(() -> {
            this.runTopMotor(INTAKE.TOP_MOTOR_OUTTAKE_SPEED);
            this.runBottomMotor(INTAKE.BOTTOM_MOTOR_OUTTAKE_SPEED);
        });
    }

    public Command stopCommand() {
        return this.run(() -> {
            topMotor.setPercentOutput(0);
            bottomMotor.setPercentOutput(0);
        });
    }

}
