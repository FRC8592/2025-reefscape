package frc.robot.subsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;

public class DeepClimb extends SubsystemBase {

    NewtonMotor deepClimbMotor;
    SparkFlexMotor deepClimbIntakeMotor;
    double motorRotations = 0;
    boolean motorRotationsSet = false;
    LinearFilter filter = LinearFilter.movingAverage(30);


    public DeepClimb() {
        deepClimbMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_MOTOR_CAN_ID, true); 
        deepClimbMotor.setIdleMode(IdleMode.kBrake);

        deepClimbIntakeMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_INTAKE_MOTOR_CAN_ID, true);
        deepClimbIntakeMotor.setIdleMode(IdleMode.kBrake);


        // deepClimbMotor.withGains(new PIDProfile().setPID(DEEP_CLIMB.DEEP_CLIMB_HOLD_P, 0, 0));
    }

    public void setDeepClimbPercentOutput(double percent){
        deepClimbMotor.setPercentOutput(percent);
        // if(percent != 0){
        //     motorRotationsSet = false;
        // }
        // else{
        //     deepClimbMotor.setPercentOutput(percent);
        //     motorRotationsSet = true;
        //     motorRotations = deepClimbMotor.getRotations();
        // }
    }

    public void setDeepClimbIntakePercentOutput(double percent){
        deepClimbIntakeMotor.setPercentOutput(percent);
    }

    // public boolean isDeepClimbDeployed() {
    //     return deepClimbMotor.getRotations() > DEEP_CLIMB.DEEP_CLIMB_START_POSITION;
    // }

    public Command setDeepClimbGrabPositionCommand(){
        return this.run(() -> setDeepClimbPercentOutput(-1)).until(
            () -> deepClimbMotor.getRotations() <= DEEP_CLIMB.DEEP_CLIMB_GRAB_POSITION
        ).finallyDo(() -> setDeepClimbPercentOutput(0));
    }


    public void stopDeepClimb() {
        setDeepClimbPercentOutput(0);
    }

    public void stopDeepIntakeClimb() {
        setDeepClimbIntakePercentOutput(0);
    }

    public Command setDeepClimbCommand(double percent){
        return this.runOnce(()->{setDeepClimbPercentOutput(percent);}); 
        // .until(() -> (
        //     deepClimbMotor.getRotations()>=DEEP_CLIMB.DEEP_CLIMB_MAX_POSITION
        //     && percent > 0
        // ))
        //.finallyDo(() -> this.setDeepClimbPercentOutput(0));
    }

    public Command setDeepClimbIntakeCommand(double percent){
        return this.run(()->{
            setDeepClimbIntakePercentOutput(percent);
        }).finallyDo(() -> this.setDeepClimbIntakePercentOutput(0));
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
        double fliteredCurrent = filter.calculate(deepClimbIntakeMotor.getOutputCurrent());

        Logger.recordOutput(SHARED.LOG_FOLDER + "Motor current", deepClimbIntakeMotor.getOutputCurrent());
        Logger.recordOutput(SHARED.LOG_FOLDER + "Winch Rotations", deepClimbMotor.getRotations());
        Logger.recordOutput(SHARED.LOG_FOLDER + "Filtered current" , fliteredCurrent);
        // if(motorRotationsSet){
        //     deepClimbMotor.setPosition(motorRotations);
        // }
        LEDs.setDeepClimb(fliteredCurrent>50);
        if(Math.abs(deepClimbMotor.getVelocityRPM())>30 && 180>Math.abs(deepClimbMotor.getVelocityRPM())){
            LEDs.setRainbow(true);
        }
        else{ 
            LEDs.setRainbow(false);
        }
    }
}
