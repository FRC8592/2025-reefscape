package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Intake extends SubsystemBase {

    NewtonMotor intakeMotor;
    private DigitalInput intakeSensor;
    public Intake() {
        intakeMotor = new KrakenX60Motor(CAN.INTAKE_MOTOR_CAN_ID, true);
        intakeSensor = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
    * Accepts the desired speed as a percentage and sets the motor to the given speed.
    * @param percent Desired speed as a percentage.
    */
    public void setIntakePercentOutput(double percent){
        intakeMotor.setPercentOutput(percent);
    }

    /**
    * Stops the intake motor by setting the percent output to 0
    */
    public void stop() {
        setIntakePercentOutput(0);
    }

    /**
     * Accepts the desired power of the motor and sets the motor to that power.
     * @param percent Desired percentage of the motor.
     * @return Returns a command to set motor power to given percentage.
     */
    public Command setIntakeCommand(double percent){
        return this.run(()->{setIntakePercentOutput(percent);});
    }

    /**
     * Stops the intake motor by setting the percent output to 0.
     * @return Returns a command to stop the intake motor.
     */
    public Command stopIntakeCommand(){
        return this.runOnce(()->{
            setIntakePercentOutput(0);
        });
    }

    /**
     * Checks to see if the beambreak detects a coral in the intake.
     * @return Returns whether there is a coral in the intake as a boolean.
     */
    public boolean robotHasCoral(){
        return intakeSensor.get();
    
    }

    public void periodic() {
        // Logs whether the robot has a coral or not.
        SmartDashboard.putBoolean("Intake|HasCoral", robotHasCoral());
        Logger.recordOutput(INTAKE.LOG_PATH+"Intake|HasCoral", robotHasCoral());
    }
}
