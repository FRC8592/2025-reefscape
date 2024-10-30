package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SparkFlexControl;

public class ClawIntake extends SubsystemBase{
    private SparkFlexControl leftMotor;
    private SparkFlexControl rightMotor;
    public ClawIntake(){
        leftMotor = new SparkFlexControl(28, false);
        leftMotor.setInverted();
        rightMotor = new SparkFlexControl(29, false);

        leftMotor.setPIDF(Constants.INTAKE_MOTOR_P, Constants.INTAKE_MOTOR_I, Constants.INTAKE_MOTOR_D, Constants.INTAKE_MOTOR_F, 0);
        rightMotor.setPIDF(Constants.INTAKE_MOTOR_P, Constants.INTAKE_MOTOR_I, Constants.INTAKE_MOTOR_D, Constants.INTAKE_MOTOR_F, 0);

        leftMotor.setPercentOutput(0);
        rightMotor.setPercentOutput(0);
    }

    public void setVelocity(double speed) {
        leftMotor.setVelocity(speed);
        rightMotor.setVelocity(speed);
    }

    public void setPercentOutput(double power) {
        leftMotor.setPercentOutput(power);
        rightMotor.setPercentOutput(power);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
    }  
}