package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.helpers.SparkFlexControl;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.INTAKE;

public class Intake extends NewtonSubsystem {

    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance == null) {
            throw new IllegalStateException("The intake subsystem must be instantiated before using it");
        }
        return instance;
     }
     public static Intake instantiate() {
        if (instance != null) {
            throw new IllegalStateException("The intake subsystem can't be instantiated twice");
        }
        instance = new Intake();
        return instance;
     }

     public IntakeCommands commands = new IntakeCommands(this);

     private SparkFlexControl rightMotor;
     private SparkFlexControl leftMotor; 

     // Used for logging
     private double targetIntakeVelocity = 0;

     private Intake() {
        rightMotor = new SparkFlexControl(0, true);
        leftMotor = new SparkFlexControl(0, true);

        rightMotor.setPIDF(INTAKE.INTAKE_MOTOR_kP, INTAKE.INTAKE_MOTOR_kI, INTAKE.INTAKE_MOTOR_kD, INTAKE.INTAKE_MOTOR_kFF, INTAKE.LEFT_SLOT_ID);
        leftMotor.setPIDF(INTAKE.INTAKE_MOTOR_kP, INTAKE.INTAKE_MOTOR_kI, INTAKE.INTAKE_MOTOR_kD, INTAKE.INTAKE_MOTOR_kFF, INTAKE.RIGHT_SLOT_ID);

        rightMotor.setCurrentLimit(0, 0);
        leftMotor.setCurrentLimit(0, 0);

     }

     public void periodic() {
        Logger.recordOutput(null, null);
        Logger.recordOutput(null, null);
     }

     public void simulationPeriodic() {
        // This method is something Martin wrote
     }

     protected void setIntakeVelocity(double velocity) {
        targetIntakeVelocity = velocity;
        rightMotor.setVelocity(velocity);
        leftMotor.setVelocity(velocity);
     }

     protected void stop() {
        targetIntakeVelocity = 0.0;
        setIntakeVelocity(targetIntakeVelocity);
     }
}
