package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Pivot extends SubsystemBase{
    public enum Positions {
        GROUND(PIVOT.GROUND_DEGREES),
        REST(PIVOT.STOW_DEGREES),
        SCORE_GRID(PIVOT.SCORE_GRID_DEGREES),
        SCORE_HIGH(PIVOT.SCORE_HIGH_DEGREES),;

        public int degrees = 0;
        Positions (int degrees){
            this.degrees = degrees;
        }
    }

    SparkFlexControl pivotMotor;
    public Pivot(){
        pivotMotor = new SparkFlexControl(CAN.PIVOT_MOTOR_CAN_ID, true);
        pivotMotor.setPIDF(PIVOT.PIVOT_kP, PIVOT.PIVOT_kI, PIVOT.PIVOT_kD, PIVOT.PIVOT_kF, 0);
        pivotMotor.setInverted();
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 75.0);
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 1);
        
        pivotMotor.setMaxVelocity(5000, 0);
        pivotMotor.setMaxAcceleration(7000, 0);
    }

    public void setMotorPower(double power) {
        pivotMotor.setPercentOutput(power);
    }
    
    public void setPosition(double pivotDegrees) {
        pivotMotor.setPositionSmartMotion((pivotDegrees/360) * PIVOT.PIVOT_GEAR_RATIO);
    }

    public void stop(){
        setMotorPower(0);
    }

    public void periodic(){
        double currentPivotDegrees = (pivotMotor.getPosition()/PIVOT.PIVOT_GEAR_RATIO)*360;

        SmartDashboard.putNumber(
            "CurrentPivotMotorRotation", 
            pivotMotor.getPosition()
        );
        SmartDashboard.putNumber(
            "CurrentPivotDegrees",
            currentPivotDegrees
        );
        SmartDashboard.putNumber(
            "Current Pivot Motor Speed",
            pivotMotor.getVelocity()
        );
        Logger.recordOutput(
            "Current Pivot Motor Speed", 
            pivotMotor.getVelocity()
        );
        /*The below if statement checks if the current degrees 
        of the pivot is at least 90 degrees and sets the motor in brake mode so it doesn't fall*/
        //Pre-conditon: Degrees of pivot must be greater than or equal to 90 degrees
        //Post-condition: Sets the motor to brake mode
        //This should be here because it needs to constantly be called
        if (currentPivotDegrees >= PIVOT.STOW_DEGREES) {
            pivotMotor.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        }
    }
}
