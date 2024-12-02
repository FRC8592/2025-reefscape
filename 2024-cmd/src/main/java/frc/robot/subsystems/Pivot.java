package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;

public class Pivot extends SubsystemBase{
    public enum Positions {
        GROUND(PIVOT.GROUND_DEGREES),
        REST(PIVOT.REST_DEGREES),
        HP_LOAD(PIVOT.HP_LOAD_DEGREES),
        SCORE_HIGH(PIVOT.SCORE_HIGH_DEGREES),;

        public int degrees = 0;
        Positions (int degrees){
            this.degrees = degrees;
        }
    }

    SparkFlexControl pivotMotor;
    public Pivot(){
        pivotMotor = new SparkFlexControl(CAN.PIVOT_MOTOR_CAN_ID, false);
        pivotMotor.setPIDF(PIVOT.PIVOT_kP, PIVOT.PIVOT_kI, PIVOT.PIVOT_kD, PIVOT.PIVOT_kF, 0);
        pivotMotor.setInverted();
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 75.0);
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 1);
    }

    public void setMotorPower(double power) {
        pivotMotor.setPercentOutput(power);
    }

    public void stop(){
        setMotorPower(0);
    }

    public void periodic(){
        SmartDashboard.putNumber(
            "CurrentPivotMotorRotation", 
            pivotMotor.getPosition()
        );
        SmartDashboard.putNumber(
            "CurrentPivotDegrees",
            (pivotMotor.getPosition()/Constants.PIVOT.PIVOT_GEAR_RATIO)*360
        );
    }
}
