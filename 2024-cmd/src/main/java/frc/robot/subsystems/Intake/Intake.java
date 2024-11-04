package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.helpers.SparkFlexControl;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.CAN;

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

     private SparkFlexControl topMotor;
     private SparkFlexControl bottomMotor; 

     // Used for logging
     private double targetIntakeVelocity = 0;

     private Intake() {
        topMotor = new SparkFlexControl(CAN.INTAKE_TOP_CAN_ID, true);
        bottomMotor = new SparkFlexControl(CAN.INTAKE_BOTTOM_CAN_ID, true);

        topMotor.setPIDF(INTAKE.INTAKE_MOTOR_kP, INTAKE.INTAKE_MOTOR_kI, INTAKE.INTAKE_MOTOR_kD, INTAKE.INTAKE_MOTOR_kFF, INTAKE.SLOT_ID);
        bottomMotor.setPIDF(INTAKE.INTAKE_MOTOR_kP, INTAKE.INTAKE_MOTOR_kI, INTAKE.INTAKE_MOTOR_kD, INTAKE.INTAKE_MOTOR_kFF, INTAKE.SLOT_ID);

        topMotor.setCurrentLimit(0, 0);
        bottomMotor.setCurrentLimit(0, 0);

     }
     @Override
     public void periodic() {
      //   Logger.recordOutput("", 0.0);
      //   Logger.recordOutput("", 0.0);
     }

     @Override
     public void simulationPeriodic() {
        // This method is something Martin wrote
     }

     protected void setIntakeVelocity(double velocity) {
        targetIntakeVelocity = velocity;
        topMotor.setVelocity(velocity);
        bottomMotor.setVelocity(velocity);
     }

     protected void stop() {
        targetIntakeVelocity = 0.0;
        setIntakeVelocity(targetIntakeVelocity);
     }
}
