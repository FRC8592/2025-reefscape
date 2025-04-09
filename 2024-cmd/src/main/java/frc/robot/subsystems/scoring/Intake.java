package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;

public class Intake extends SubsystemBase {

    private NewtonMotor intakeMotor;
    // private DigitalInput intakeSensor;
    private LaserCan intakeSensor;

    public Intake() {
        intakeMotor = new KrakenX60Motor(CAN.INTAKE_MOTOR_CAN_ID, true);
        // intakeSensor = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
        intakeSensor = new LaserCan(CAN.INTAKE_BEAM_BREAK_CAN_ID);
        try {
            intakeSensor.setRangingMode(LaserCan.RangingMode.SHORT); 
            intakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            intakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.err.println("Configuration failed! " + e);
        }
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
        Measurement measurement = intakeSensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm < INTAKE.INTAKE_BEAM_BREAK_THRESHOLD_MM;
        } else {
            return false;
        }
    
    }

    public void periodic() {
        // Logs whether the robot has a coral or not.
        SmartDashboard.putBoolean("Intake|HasCoral", robotHasCoral());
        // SmartDashboard.putString("Intake|CoralDistance", intakeSensor.getMeasurement().toString());
        Logger.recordOutput(INTAKE.LOG_PATH+"Intake|HasCoral", robotHasCoral());
        Logger.recordOutput(INTAKE.LOG_PATH+"Intake|MotorSpeed",intakeMotor.getVelocityRPM());
    }
}
