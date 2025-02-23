package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class DeepClimb extends SubsystemBase {

    NewtonMotor deepClimbMotor;
    NewtonMotor deepClimbIntakeMotor;

    public DeepClimb() {
        deepClimbMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_MOTOR_CAN_ID, true); //TODO: add back spark flex motor
        deepClimbMotor.setIdleMode(IdleMode.kBrake);

        deepClimbIntakeMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_INTAKE_MOTOR_CAN_ID, true);
        deepClimbIntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setDeepClimbPercentOutput(double percent){
        deepClimbMotor.setPercentOutput(percent);
    }

    public void setDeepClimbIntakePercentOutput(double percent){
        deepClimbIntakeMotor.setPercentOutput(percent);
    }

    public void stopDeepClimb() {
        setDeepClimbPercentOutput(0);
    }

    public void stopDeepIntakeClimb() {
        setDeepClimbIntakePercentOutput(0);
    }

    public Command setDeepClimbCommand(double percent){
        return this.run(()->{setDeepClimbPercentOutput(percent);});
    }

    public Command setDeepClimbIntakeCommand(double percent){
        return this.run(()->{setDeepClimbIntakePercentOutput(percent);});
    }

    public Command stopDeepClimbCommand(){
        return this.runOnce(()->{
            setDeepClimbPercentOutput(0);
        });
    }

    public Command stopDeepClimbIntakeCommand(){
        return this.runOnce(()->{
            setDeepClimbIntakePercentOutput(0);
        });
    }

    public void periodic() {
        
    }
}
