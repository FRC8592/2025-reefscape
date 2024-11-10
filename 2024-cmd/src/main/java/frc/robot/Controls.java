package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CONTROLLERS;

public final class Controls {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    /**
     * Enum for the different sets of controls (different drivers,
     * different DS configurations, etc)
     */
    protected enum ControlSets{
        MAIN_TELEOP,
        DISABLED,
    }

    protected static DoubleSupplier driveTranslateX = () -> 0;
    protected static DoubleSupplier driveTranslateY = () -> 0;
    protected static DoubleSupplier driveRotate = () -> 0;

    protected static Trigger slowMode = new Trigger(() -> false);
    protected static Trigger robotRelative = new Trigger(() -> false);
    protected static Trigger zeroGryoscope = new Trigger(() -> false);

    protected static Trigger snapForward = new Trigger(() -> false);
    protected static Trigger snapBack = new Trigger(() -> false);
    protected static Trigger snapLeft = new Trigger(() -> false);
    protected static Trigger snapRight = new Trigger(() -> false);
    
    protected static Trigger intake = new Trigger(() -> false);
    protected static Trigger score = new Trigger(() -> false);

    protected static Trigger stow = new Trigger(() -> false);

    /**
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
        switch (set) {
            default: case DISABLED:
                driveTranslateX = () -> 0;
                driveTranslateY = () -> 0;
                driveRotate = () -> 0;

                slowMode = new Trigger(() -> false);
                robotRelative = new Trigger(() -> false);
                zeroGryoscope = new Trigger(() -> false);

                snapForward = new Trigger(() -> false);
                snapBack = new Trigger(() -> false);
                snapLeft = new Trigger(() -> false);
                snapRight = new Trigger(() -> false);

                intake = new Trigger(() -> false);
                score = new Trigger(() -> false);

                stow = new Trigger(() -> false);
                break;

            case MAIN_TELEOP:
                driveTranslateX = () -> -driverController.getLeftX();
                driveTranslateY = () -> -driverController.getLeftY();
                driveRotate = () -> -driverController.getRightX();

                intake = driverController.leftTrigger(0.2);
                score = driverController.rightTrigger(0.2);

                stow = driverController.leftBumper();

                slowMode = driverController.rightBumper();
                robotRelative = driverController.a();
                zeroGryoscope = driverController.back();

                snapForward = driverController.pov(0);
                snapBack = driverController.pov(180);
                snapLeft = driverController.pov(90);
                snapRight = driverController.pov(270);
                break;
        }
    }
}
