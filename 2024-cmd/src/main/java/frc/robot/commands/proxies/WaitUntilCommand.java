package frc.robot.commands.proxies;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;

public class WaitUntilCommand extends WrapperCommand {
    public WaitUntilCommand(Command command, BooleanSupplier endCondition) {
        super(new WaitCommand(Double.POSITIVE_INFINITY).until(endCondition).andThen(command));
    }
}
