package frc.robot.subsystems.shooter;

import frc.robot.commands.proxies.NewtonCommand;
import frc.robot.subsystems.NewtonSubsystem;

public class Shooter extends NewtonSubsystem {

    private static Shooter Instance = null; 

    public static Shooter getInstance() {
        if (Instance == null) {
            throw new IllegalStateException("The outtake subsystem must be instantiated before using it");
        }
        return Instance;
     }
     public static Shooter instantiate() {
        if (Instance != null) {
            throw new IllegalStateException("The outtake subsystem can't be instantiated twice");
        }
        Instance = new Shooter();
        return Instance;
     }

     //public ShooterCommands commands = new ShooterCommands(this);

     private SparkFlexControl shooterMotor;

     private double targetShooterVelocity = 0;


     private Shooter() {
        shooterMotor = new SparkFlexControl(0, false);
     }

     public void periodic() {
     }

     public void simulationPeriodic() {
        // This method is something Martin wrote
     }

     public void spinShooter(double velocity) {
        targetShooterVelocity = velocity;
        shooterMotor.setVelocity(velocity);
     }

     protected void stop() {
        targetShooterVelocity = 0.0;
        spinShooter(targetShooterVelocity);
     }

    }
