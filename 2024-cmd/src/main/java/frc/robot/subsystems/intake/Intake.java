package frc.robot.subsystems.intake;

import frc.robot.subsystems.NewtonSubsystem;

public class Intake extends NewtonSubsystem{
    private static Intake instance = null;
    public static Intake getInstance(){
        if(instance == null){
            throw new IllegalStateException("The Intake subsystem must be instantiated before attempting to use it");
        }
        return instance;
    }
    public static Intake instantiate(){
        if(instance != null){
            throw new IllegalStateException("The Intake subsystem can't be instantiated twice");
        }
        instance = new Intake();
        return instance;
    }

    public IntakeCommands commands = new IntakeCommands(this);

    public void stop(){
    }

}
