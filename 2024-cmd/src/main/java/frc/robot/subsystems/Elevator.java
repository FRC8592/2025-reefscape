package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor;
    private TalonFX wristMotor;
    private double targetElevatorPos;
    private double targetWristPos;

    public Elevator(){
        elevatorMotor = new TalonFX(0);
        wristMotor = new TalonFX(0);
    }

    public void setElevatorPos(double position){
        targetElevatorPos = position;
    }

    public void setWristPos(double position){
        targetWristPos = position;
    }

    public double getElevatorPos(){
        return 0;
    }

    public double getWristPos(){
        return 0;
    }
    
    public void periodic(){

    }


}
