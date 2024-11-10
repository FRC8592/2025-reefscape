package frc.robot.subsystems.shooter;

import edu.wpl.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemCommands;

public class Shooter extends NewtonSubsystemCommand {
    private Shooter shooter;

    public ShooterCommands(Shooter shootNote){
        shooter = shootNote;
    }

    public Command shootCommand() {
        return shoot.run(( -> {
            shoot.spinShooter(Constants.SHOOTER.SHOOTER_VELOCITY);
        }));
    }
    
    @override
    public command stopCommand(){
        
    }

}