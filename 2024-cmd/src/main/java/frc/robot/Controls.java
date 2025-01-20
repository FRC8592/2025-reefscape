package frc.robot;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CONTROLLERS;
import lib.team8592.logging.SmartLogger;

public final class Controls {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );

    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    /**
     * Enum for the different sets of controls 
     * (different drivers, different DS configurations, etc)
     */
    protected enum ControlSets{
        SINGLE_DRIVER,
        DUAL_DRIVER,
        DEMO_CONTROLS
    }

    private static SmartLogger logger;

    private static boolean logToShuffleboard = false;
    private static boolean loggingEnabled = false;

    protected static DoubleSupplier driveTranslateX = () -> 0;
    protected static DoubleSupplier driveTranslateY = () -> 0;
    protected static DoubleSupplier driveRotate = () -> 0;

    protected static Trigger slowMode = new Trigger(() -> false);
    protected static Trigger robotRelative = new Trigger(() -> false);
    protected static Trigger zeroGryoscope = new Trigger(() -> false);

    protected static Trigger snapNorth = new Trigger(() -> false);
    protected static Trigger snapSouth = new Trigger(() -> false);
    protected static Trigger snapWest = new Trigger(() -> false);
    protected static Trigger snapEast = new Trigger(() -> false);

    protected static Trigger intakeCoral = driverController.leftTrigger();
    protected static Trigger intakeAlgae = driverController.leftTrigger();
    protected static Trigger scorePiece = driverController.rightTrigger();
    protected static Trigger alignToLeftBranch = driverController.x();
    protected static Trigger alignToRightBranch = driverController.b();
    protected static Trigger stow = driverController.button(100);
    protected static Trigger goToPrimedPosition = driverController.button(100);

    protected static Trigger primeL1 = operatorController.button(1);
    protected static Trigger primeL2 = operatorController.button(2);
    protected static Trigger primeL3 = operatorController.button(3);
    protected static Trigger primeL4 = operatorController.button(4);
    protected static Trigger primeAlgaeL2 = operatorController.button(5);
    protected static Trigger primeAlgaeL3 = operatorController.button(6);
    protected static Trigger primeNet = operatorController.button(7);
    protected static Trigger primeProcessor = operatorController.button(8);

    /**
     * Sets the controls for the drivebase movement
     */
    private static void applyDrivetrainControls() {
        driveTranslateX = () -> -driverController.getLeftX();
        driveTranslateY = () -> -driverController.getLeftY();
        driveRotate = () -> -driverController.getRightX();

        slowMode = driverController.rightBumper();
        robotRelative = driverController.leftBumper();
        zeroGryoscope = driverController.back();

        snapNorth = driverController.pov(0);
        snapSouth = driverController.pov(180);
        snapWest = driverController.pov(270);
        snapEast = driverController.pov(90);
    }

    /**
     * Change the variables in the Controls class to match the specified
     * control set. Note that this doesn't edit or remove bindings.
     *
     * @param set the control set to apply
     */
    protected static void applyControlSet(ControlSets set){
        Controls.applyDrivetrainControls();

        // Add controls that do not rely on control set below

        // Use the controls set if we have differentiating inputs for a certain control
        // i.e. if single driver intaking is driver left trigger whereas 
        //        dual driver has operator left trigger
        switch(set) {
            case SINGLE_DRIVER:

                break;
            default: case DUAL_DRIVER:

                break;
        }
        
    }

    public static void initializeShuffleboardLogs(boolean logToShuffleboard) {
        Controls.logToShuffleboard = logToShuffleboard;
        if (!logToShuffleboard) return;
        
        Controls.loggingEnabled = true;
        Controls.logger = new SmartLogger("Controls");
        Controls.logger.enable();
    }

    public static void logControlsToShuffleboard() {
        if (!Controls.logToShuffleboard) return; // Don't log if we don't want to log
        
        if (!Controls.loggingEnabled) { // If not already enabled, enable it
            initializeShuffleboardLogs(true);
        }

        for (Field field : Controls.class.getDeclaredFields()) {
            try {
                logger.log(field.getName(), ((Trigger)field.get(null)).getAsBoolean());
            } catch (Exception e) {}
        }
    }

    protected static CommandXboxController getDriver() {
        return driverController;
    }

    protected static CommandXboxController getOperator() {
        return operatorController;
    }
}
