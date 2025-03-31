package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot.CurrentRobot;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
        public static final CurrentRobot CURRENT_ROBOT = CurrentRobot.PERRY;
        public static final boolean IS_RIPTIDE = CURRENT_ROBOT == CurrentRobot.RIPTIDE;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int CORAL_SELECTOR_PORT = 2;
    }

    public final class CAN {
        public static final int INTAKE_MOTOR_CAN_ID = 45;
        public static final int PDH_CAN_ID = 1;
        public static final int DEEP_CLIMB_MOTOR_CAN_ID = 51;
        public static final int DEEP_CLIMB_INTAKE_MOTOR_CAN_ID = 29;

        public static final int BACK_EXTENSION_MOTOR_CAN_ID = 44;
        public static final int FORWARD_EXTENSION_MOTOR_CAN_ID = 43;
        public static final int CLOCK_ARM_CAN_ID = 41;
        public static final int WRIST_CAN_ID = 40;
        public static final int INTAKE_BEAM_BREAK_CAN_ID = 60;
    }

    public final class CORAL_ALIGN {
        public static final double OFFSET_DEPTH = 0.40; // Drivers requested for the robot to be as close to the april tag as possible
        public static final double OFFSET_LEFT_METERS = -0.137;
        public static final double OFFSET_RIGHT_METERS = 0.213;
        public static final double ROT_OFFSET = 0d;
        public static final double SPEED_SCALE = 1.0;
        public static final double SPEED_MAX = 0.2; // originally 0.65

        public static final int MAX_LOCK_LOSS_TICKS = 20;

        public static final Transform3d CAMERA_OFFSETS = (
            SHARED.IS_RIPTIDE
            ? /* RIPRIDE: */ new Transform3d(new Translation3d(0.21, 0.215, 0.17), new Rotation3d(0, Math.toRadians(-12), 0))
            : /* PERRY: */   new Transform3d(new Translation3d(0.17145, 0.20955, 0.2286), new Rotation3d(0, Math.toRadians(-12), 0))
        );
        public static final Transform3d CAMERA_2_OFFSETS = (
            new Transform3d(new Translation3d(0.215, -0.32, 0.24), new Rotation3d(0, Math.toRadians(-12), Math.toRadians(5)))
        );

        public static final String CAMERA_NAME = (
            "Arducam_OV9782_D"
        );
        public static final String CAMERA_2_NAME = (
            "Arducam_OV9782_B"
        );
    
        public static final int[] BLUE_REEF_TAGS = {17, 18, 19, 20, 21, 22};
        public static final int[] RED_REEF_TAGS = {6, 7, 8, 9, 10, 11};

        public static final int[] BLUE_HPLAYER_TAGS = {12, 13};
        public static final int[] RED_HPLAYER_TAGS = {1, 2};

        public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 1d;

    }
    public final class NAVIGATION {
        public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    }
   
    
    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Intake/";
        public static final int INTAKE_BEAM_BREAK_THRESHOLD_MM = 20;
    }

    public final class SCORING {

        public static final double SAFE_ELEVATOR_HEIGHT = 14d;
        public static final double SAFE_ARM_POS = 55d;
        public static final double MAX_RESTRICTED_WRIST = 30d;


        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Scoring/";
    }

    
    public final class SWERVE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

        //TODO: Double check that these PID constants still work
        public static final double SNAP_TO_kP = 3.7;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;

        public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
        public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(4, 2);
        public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);

        public static final boolean INVERT_LEFT_SIDE = false;
        public static final boolean INVERT_RIGHT_SIDE = true;

        public static final double SIMULATED_STEER_INERTIA = 0.00001;
        public static final double SIMULATED_DRIVE_INERTIA = 0.06;
        public static final double SIMULATION_LOOP_PERIOD = 0.005;
        public static final double STEER_FRICTION_VOLTAGE = 0.25;
        public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

        //TODO: Tone these down appropriately as per BB rules
        public static final double TRANSLATE_POWER_FAST = 1.0; 
        public static final double ROTATE_POWER_FAST = 0.75; 
        public static final double TRANSLATE_POWER_SLOW = 0.5;
        public static final double ROTATE_POWER_SLOW = 0.3;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 1.2;

        public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

        //TODO: Double check that these still work
        // public static final double PATH_FOLLOW_TRANSLATE_kP = 8d; // Was 8 in the last test
        // public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        // public static final double PATH_FOLLOW_TRANSLATE_kD = 0.2d;

        // //TODO: Double check that these still work
        // public static final double PATH_FOLLOW_ROTATE_kP = 8;
        // public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        // public static final double PATH_FOLLOW_ROTATE_kD = 0.1;

        public static final double PATH_FOLLOW_TRANSLATE_kP = 8d; // Was 8 in the last test
        public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_ROTATE_kP = 12;
        public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        public static final double PATH_FOLLOW_ROTATE_kD = 0;

        public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
        public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 4 * Math.PI;

        public static final double PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE = 0.01; // Meters
        public static final double PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE = 0.02;

        public static final double PATH_FOLLOW_ROTATE_POSITION_TOLERANCE = 0.05; // Radians
        public static final double PATH_FOLLOW_ROTATE_VELOCITY_TOLERANCE = 0.03;
    }

    public final class ROBOT {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }

    public class SUPPLIERS{
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
    }

    public class ELEVATOR{
        public static final String EXTENSION_LOG_PATH = SHARED.LOG_FOLDER+"/Extension/";

        public static final double EXTENSION_GEAR_RATIO = 0.25;
        public static final double EXTENSION_DRUM_DIAMTER_INCHES = 1;

        public static final double EXTENSION_INCHES_MAX = SHARED.IS_RIPTIDE ? /*RIPTIDE*/ 19.5: /*PERRY*/ 19.6; //this is in inches
        public static final double EXTENSION_INCHES_MIN = SHARED.IS_RIPTIDE ? /*RIPTIDE*/ 0.5 : /*PERRY*/ 0.5;

        public static final double EXTENSION_POSITION_TOLERANCE = 0.1;

        public static final int ELEVATOR_CURRENT_LIMIT = 40;//amps

        public static final double ELEVATOR_MAX_ACCELERATION = 250;
        public static final double ELEVATOR_MAX_VELOCITY = 100; //formerly 100

        public static final double ELEVATOR_POSITION_P = SHARED.IS_RIPTIDE? /*RIPTIDE: */3.5: /*PERRY: */3.5;
        public static final double ELEVATOR_POSITION_I = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_D = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_S = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_V = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_A = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;

        public static final double ELEVATOR_VELOCITY_P = SHARED.IS_RIPTIDE? /*RIPTIDE: */0.001: /*PERRY: */0.001;
        public static final double ELEVATOR_VELOCITY_I = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_VELOCITY_D = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_VELOCITY_S = SHARED.IS_RIPTIDE? /*RIPTIDE: */0.1: /*PERRY: */0.1;
    }

    public class ARM{
        public static final String CLOCK_ARM_LOG_PATH = SHARED.LOG_FOLDER+"/Clock Arm/";
        
        public static final double CLOCK_ARM_GEAR_RATIO = 1/180.0;

        public static final double ARM_ANGLE_DEGREES_MIN = SHARED.IS_RIPTIDE? -2 : -2;
        public static final double ARM_ANGLE_DEGREES_MAX = SHARED.IS_RIPTIDE? 180 : 180;

        public static final double SAFE_ARM_TO_ROTATE_WRIST = SHARED.IS_RIPTIDE ? 75 : 50;

        public static final double CLOCK_ARM_POSITION_TOLERANCE = 2.0;

        public static final int ARM_CURRENT_LIMIT = 40;//amps

        public static final double ARM_MAX_ACCELERATION = 250;
        public static final double ARM_MAX_VELOCITY = 100; //previously 100

        public static final double ARM_P = SHARED.IS_RIPTIDE? /*RIPTIDE: */3: /*PERRY: */3;
        public static final double ARM_I = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ARM_D = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ARM_S = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ARM_V = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ARM_A = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
    }

    public class WRIST{
        public static final String WRIST_LOG_PATH = SHARED.LOG_FOLDER+"/Wrist/";

        public static final double WRIST_GEAR_RATIO = 1/75.0;

        public static final double WRIST_ANGLE_DEGREES_MIN = SHARED.IS_RIPTIDE? -225d : -225d;
        public static final double WRIST_ANGLE_DEGREES_MAX = SHARED.IS_RIPTIDE? 213 : 213;

        public static final double WRIST_POSITION_TOLERANCE = 5.0;

        public static final int WRIST_CURRENT_LIMIT = 60;//amps

        public static final double WRIST_MAX_ACCELERATION = 400;
        public static final double WRIST_MAX_VELOCITY = 100; //used to be 100

        public static final double WRIST_P = SHARED.IS_RIPTIDE? /*RIPTIDE: */3: /*PERRY: */3;
        public static final double WRIST_I = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double WRIST_D = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double WRIST_S = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double WRIST_V = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double WRIST_A = SHARED.IS_RIPTIDE? /*RIPTIDE: */0: /*PERRY: */0;
    }

    public final class LEDS{
        public static final Color TEAL = new Color(0, 64, 192);
        public static final Color ORANGE = new Color(192, 64, 0);
        public static final Color WHITE = new Color(255, 255, 255);
        public static final Color GREEN = new Color(0,255,0);
        public static final Color RED = new Color(255, 0, 0);
        public static final Color OFF = new Color(0, 0, 0);
        public static final Color YELLOW = new Color(255,255,0);
        public static final Color PURPLE = new Color(255,0,255);
        public static final int LED_STRIP_LENGTH = 52;
        public static final int LED_CANDLE_COUNT= 8; 
        public static final int FULL_LED_COUNT = LED_STRIP_LENGTH+LED_CANDLE_COUNT;
    }

    public final class DEEP_CLIMB{
        public static final double DEEP_CLIMB_GRAB_POSITION = -280;
        public static final double DEEP_CLIMB_MAX_POSITION = -25;
        public static final double DEEP_CLIMB_START_POSITION = 0;
        public static final double DEEP_CLIMB_HOLD_P = 0.1;
    }
}
