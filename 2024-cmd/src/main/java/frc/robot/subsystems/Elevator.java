package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private TalonFX leftElevatorMotor;
    private TalonFX rightElevatorMotor;
    private TalonFX pivotMotor;
    private double targetElevatorPos;
    private double targetPivotPos;

    public Elevator(){
        leftElevatorMotor = new TalonFX(0);
        rightElevatorMotor = new TalonFX(0);
        pivotMotor = new TalonFX(0);
    }

    public void setElevatorPos(double position){
        targetElevatorPos = position;
    }

    public void setPivotPos(double position){
        targetPivotPos = position;
    }

    public double getElevatorPos(){
        return 0;
    }

    public double getPivotPos(){
        return 0;
    }

    public void stopElevator(){
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

    public void stopPivot(){
        pivotMotor.stopMotor();
    }
    
    public void periodic(){

    }


}
