package frc.robot.subsystems.pivot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkMaxControl;
import frc.robot.subsystems.NewtonSubsystem;

public class Pivot extends NewtonSubsystem {
    
    private static Pivot instance = null;
    private SparkMaxControl pivotMotor;
    private double desiredPivotAngle;
    
    public PivotCommands commands;

    public static Pivot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("The Pivot subsystem must be instantiated before using it");
        }
        return instance;
     }
     public static Pivot instantiate() {
        if (instance != null) {
            throw new IllegalStateException("The Pivot subsystem can't be instantiated twice");
        }
        instance = new Pivot();
        return instance;
     }
    
    private Pivot(){
        pivotMotor = new SparkMaxControl(CAN.PIVOT_MOTOR_CAN_ID, false);
        
        commands = new PivotCommands(this);

        desiredPivotAngle = 0;
    }
    
   @Override
   public void periodic(){
    SmartDashboard.putNumber("Desired pivot angle", 0);
   }
    
    protected void dropPivot(){
        desiredPivotAngle = PIVOT.PIVOT_DROP_DEGREES;
        double rotations = pivotDegreesToRotations(desiredPivotAngle);
        pivotMotor.setPosition(rotations);
    }

    protected void raisePivot(){
        desiredPivotAngle = PIVOT.PIVOT_RAISE_DEGREES;
        double rotations = pivotDegreesToRotations(desiredPivotAngle);
        pivotMotor.setPosition(-rotations);
    }


    @Override
    protected void stop() {
        pivotMotor.setVelocity(0);
    }
    
    private double pivotDegreesToRotations(double degrees) {
        return (degrees / 360.0) * PIVOT.PIVOT_GEAR_RATIO;
    }
}
