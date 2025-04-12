package frc.robot.subsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
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

    }

    public void deployDeepClimb(){
        deepClimbMotor.setPercentOutput(-1);
    }

    public void liftDeepClimb(){
        deepClimbMotor.setPercentOutput(1);
    }

    public void stopDeepClimb() {
        deepClimbMotor.setPercentOutput(0);
    }

    public void runIntake(){
        deepClimbIntakeMotor.setPercentOutput(-1);
    }

    public void stopIntake() {
        deepClimbIntakeMotor.setPercentOutput(0);
    }

    public boolean atClimbPos(){ 
        return Utils.isWithin(deepClimbMotor.getRotations(), Constants.DEEP_CLIMB.DEEP_CLIMB_GRAB_POSITION, Constants.DEEP_CLIMB.DEEP_CLIMB_POSITION_TOLERANCE);
    }

    public Command deployDeepClimbCommand(){
        return this.runOnce(()->{
            deployDeepClimb();
        });
    }

    public Command liftDeepClimbCommand(){
        return this.runOnce(()->{
            liftDeepClimb();
        });
    }

    public Command stopDeepClimbCommand(){
        return this.runOnce(()->{
            stopDeepClimb();
        });
    }

    public Command runDeepClimbIntakeCommand(){
        return this.runOnce(()->{
            runIntake();
        });
    }
    
    public Command stopDeepClimbIntakeCommand(){
        return this.runOnce(()->{
            stopIntake();
        });
    }

    public Command autoDeployDeepClimbCommand(){
        return deployDeepClimbCommand().andThen(new WaitUntilCommand(() -> atClimbPos())).finallyDo(()->stopDeepClimb());
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
