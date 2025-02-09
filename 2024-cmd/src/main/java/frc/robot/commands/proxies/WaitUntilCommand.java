package frc.robot.commands.proxies;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class WaitUntilCommand extends WrapperCommand{
    
    public WaitUntilCommand(Command command, BooleanSupplier supplier){
        super(new WaitCommand(Double.POSITIVE_INFINITY).until(supplier).andThen(command));
    }
}
