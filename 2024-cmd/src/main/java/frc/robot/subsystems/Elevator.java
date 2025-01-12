package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private TalonFX leftElevatorMotor;
    private TalonFX rightElevatorMotor;
    private TalonFX pivotMotor;
    private double targetElevatorPos;
    private double targetPivotPos;

    /***
     * Instantiate elevator and pivot motors
     */
    public Elevator(){
        leftElevatorMotor = new TalonFX(0);
        rightElevatorMotor = new TalonFX(0);
        pivotMotor = new TalonFX(0);
    }
    /***
     * Sets target elevator position 
     * @param position in inches
     */
    public void setElevatorPos(double position){
        
        targetElevatorPos = position;
    }
    /***
     * Sets target pivot position 
     * @param position in inches
     */
    public void setPivotPos(double position){
        targetPivotPos = position;
    }
    /***
     * Gets the elevator position
     * @return elevator position in inches
     */
    public double getElevatorPos(){
        return 0;
    }
    /***
     * Gets the pivot position
     * @return pivot position in inches
     */
    public double getPivotPos(){
        return 0;
    }
    /***
     * Stops the elevator motors
     */
    public void stopElevator(){
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }
    /***
     * Stops the pivot motor
     */
    public void stopPivot(){
        pivotMotor.stopMotor();
    }
    /***
     * Override method for periodic
     */
    public void periodic(){

    }


}
