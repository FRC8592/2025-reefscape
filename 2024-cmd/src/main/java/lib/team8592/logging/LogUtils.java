package lib.team8592.logging;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Robot;

public class LogUtils {
    public static class LogConstants {
        public final String game, year, robot, team;
        public LogConstants(String game, String year, String robot, String team) {
            this.game = game;
            this.year = year;
            this.robot = robot;
            this.team = team;
        }
    }

    public static class WidgetProfile {
        public int height = 0;
        public int width = 0;
        public int column = 0;
        public int row = 0;

        public WidgetProfile withPosition(int column, int row) {
            this.column = column;
            this.row = row;
            return this;
        }

        public WidgetProfile withSize(int width, int height) {
            this.width = width;
            this.height = height;
            return this;
        }
    }
    
    public static void initialize(LogConstants constants, boolean isSimulation) {
        
        Logger.recordMetadata("Game", constants.game);
        Logger.recordMetadata("Year", constants.year);
        Logger.recordMetadata("Robot", constants.robot);
        Logger.recordMetadata("Team", constants.team);

        if (Robot.isReal()) {
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            Logger.addDataReceiver(new WPILOGWriter());
        }
        
        Logger.start();
    }

    public static void logToSmartDashboard(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void logToSmartDashboard(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    public static void logToSmartDashboard(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void addSendable(Sendable sendable) {
        SmartDashboard.putData(sendable);
    }

    public static void addSendable(String tabName, Sendable sendable) {
        Shuffleboard.getTab(tabName).add(sendable);
    }

    public static void addSendable(String tabName, Sendable sendable, WidgetProfile constants) {
        Shuffleboard.getTab(tabName).add(sendable).withPosition(constants.column, constants.row);
    }

    public static <T> SendableChooser<T> createSendableChooserWithDefault(T defaultChoice, List<T> choices) {
        SendableChooser<T> chooser = new SendableChooser<>();
        chooser.setDefaultOption(defaultChoice.toString(), defaultChoice);
        for (T choice : choices) {
            chooser.addOption(choice.toString(), choice);
        }
        return chooser;
    }
}