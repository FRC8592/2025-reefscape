package frc.robot;
public class Intake {
    public CANSparkFlex rightMotor;
    public CANSparkFlex leftMotor;
    public Intake(int rightmotorCanID, int leftMotorCanID) {
        rightMotor = new CANSparkFlex(rightMotorCanID, MotorType.kBrushless);
        leftMotor = new CANSparkFlex(leftMotorCanID, Motor.Type.kBrushless);
    
    }
}
