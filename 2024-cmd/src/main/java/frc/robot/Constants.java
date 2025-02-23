package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
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
        public static final CurrentRobot CURRENT_ROBOT = CurrentRobot.RIPTIDE;
        public static final boolean IS_RIPTIDE = CURRENT_ROBOT == CurrentRobot.RIPTIDE;
    }
    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int CORAL_SELECTOR_PORT = 2;

        // Assignments for coralController buttons
        public static final int CORAL_CONTROLLER_L1 = 7;
        public static final int CORAL_CONTROLLER_L2 = 8;
        public static final int CORAL_CONTROLLER_L3 = 2;
        public static final int CORAL_CONTROLLER_L4 = 3;
        public static final int CORAL_CONTROLLER_R1 = 5;
        public static final int CORAL_CONTROLLER_R2 = 6;
        public static final int CORAL_CONTROLLER_R3 = 1;
        public static final int CORAL_CONTROLLER_R4 = 4;
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
    }

    public final class POWER {
        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_DRIVE_CURRENT_LIMIT = 80;
        public static final int SWERVE_STEER_CURRENT_LIMIT = 40;
    }

    public final class CORAL_ALIGN {
        public static final double X_KP = 1; // Used to be 1. Currently testing x_KP of 0
        public static final double X_KI = 0;
        public static final double X_KD = 0.01;

        public static final double Y_KP = 0.8; // Originally set to 0.5
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double ROT_KP = 0.01; // 0.015 was the original value but set to 0 for testing y axis
        public static final double ROT_KI = 0;
        public static final double ROT_KD = 0.0001;

        public static final double OFFSET_DEPTH = 0.45; // Drivers requested for the robot to be as close to the april tag as possible
        public static final double OFFSET_LEFT_METERS = -0.175;
        public static final double OFFSET_RIGHT_METERS = 0.175;
        public static final double ROT_OFFSET = 0d;
        public static final double SPEED_SCALE = 1.0;
        public static final double SPEED_MAX = 0.2; // originally 0.65

        public static final int MAX_LOCK_LOSS_TICKS = 20;

        public static final Transform3d CAMERA_OFFSETS = (
            SHARED.IS_RIPTIDE
            ? /* RIPRIDE: */ new Transform3d(new Translation3d(0.21, 0.215, 0.17), new Rotation3d(0, Math.toRadians(-12), 0))
            : /* PERRY: */   new Transform3d(new Translation3d(0.21, 0.215, 0.17), new Rotation3d(0, Math.toRadians(-12), 0))
        );

        public static final String CAMERA_NAME = (
            SHARED.IS_RIPTIDE ? "Arducam_OV9072_B" : "Arducam_OV9072_D"
        );

        public static final Pose3d SOUTH_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(18).get();
        public static final Pose3d SOUTH_WEST_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(19).get();
        public static final Pose3d NORTH_WEST_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(20).get();
        public static final Pose3d NORTH_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(21).get();
        public static final Pose3d NORTH_EAST_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(22).get();
        public static final Pose3d SOUTH_EAST_BLUE_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(17).get();

        public static final Pose3d SOUTH_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(7).get();
        public static final Pose3d SOUTH_WEST_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(6).get();
        public static final Pose3d NORTH_WEST_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(11).get();
        public static final Pose3d NORTH_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(10).get();
        public static final Pose3d NORTH_EAST_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(9).get();
        public static final Pose3d SOUTH_EAST_RED_POSE = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(8).get();
    
        public static final int[] BLUE_REEF_TAGS = {17, 18, 19, 20, 21, 22};
        public static final int[] RED_REEF_TAGS = {6, 7, 8, 9, 10, 11};

        public static final int[] BLUE_HPLAYER_TAGS = {12, 13};
        public static final int[] RED_HPLAYER_TAGS = {1, 2};

    }
    public final class NAVIGATION {
        public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.2;
    }
   
    
    public final class INTAKE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Intake/";
        public static final int INTAKE_BEAM_BREAK_DIGITAL_ID = 9;

    }

    public final class SCORING {
    
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Scoring/";
    }

    
    public final class SWERVE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

        public static final double STEER_P = 100;
        public static final double STEER_I = 0;
        public static final double STEER_D = 0.2;
        public static final double STEER_S = 0;
        public static final double STEER_V = 1.5;
        public static final double STEER_A = 0;

        public static final double DRIVE_P = 3;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_S = 0;
        public static final double DRIVE_V = 0;
        public static final double DRIVE_A = 0;

        //TODO: Double check that these PID constants still work
        public static final double SNAP_TO_kP = 3.7;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;

        public static final int STEER_STATOR_CURRENT_LIMIT = 60;

        public static final int CALCULATED_SLIP_CURRENT = 150;

        public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
        public static final double MAX_TRANSLATIONAL_ACCELERATION = 2;
        public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(4, 2);
        public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);
        public static final double COUPLING_GEAR_RATIO = 3.5714285714285716;
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final double WHEEL_RADIUS_INCHES = 2;

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
        public static final double TRANSLATE_POWER_SLOW = 0.15;
        public static final double ROTATE_POWER_SLOW = 0.15;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 1.2;

        public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);


        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = 0.062255859375;
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.37890625;
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.0029296875;
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.293701171875;

        public static final boolean BLACK_FRONT_LEFT_STEER_INVERT = true;
        public static final boolean ORANGE_FRONT_RIGHT_STEER_INVERT = true;
        public static final boolean TEAL_BACK_LEFT_STEER_INVERT = true;
        public static final boolean WHITE_BACK_RIGHT_STEER_INVERT = true;


        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_X_POSITION = 8.375;
        public static final double BLACK_FRONT_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double ORANGE_FRONT_RIGHT_X_POSITION = 8.375;
        public static final double ORANGE_FRONT_RIGHT_Y_POSITION = -8.375;

        //TODO: Set these
        public static final double TEAL_BACK_LEFT_X_POSITION = -8.375;
        public static final double TEAL_BACK_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double WHITE_BACK_RIGHT_X_POSITION = -8.375;
        public static final double WHITE_BACK_RIGHT_Y_POSITION = -8.375;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_TRANSLATE_kP = 8d;
        public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_ROTATE_kP = 6d;
        public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        public static final double PATH_FOLLOW_ROTATE_kD = 0.1;

        public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
        public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 4 * Math.PI;

        public static final double PATH_FOLLOW_POSITION_TOLERANCE = 0.01;
        public static final double PATH_FOLLOW_VELOCITY_TOLERANCE = 0.01;
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
        public static final double EXTENSION_DRUM_DIAMTER_INCHES = 1.0;

        public static final double EXTENSION_INCHES_MAX = SHARED.IS_RIPTIDE?19.5:19.5; //this is in inches
        public static final double EXTENSION_INCHES_MIN = SHARED.IS_RIPTIDE?0.5:0.5;

        public static final double EXTENSION_POSITION_TOLERANCE = 1.0;

        public static final int ELEVATOR_CURRENT_LIMIT = 40;//amps

        public static final double ELEVATOR_MAX_ACCELERATION = 300;
        public static final double ELEVATOR_MAX_VELOCITY = 75; //formerly 75

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

        public static final double ARM_ANGLE_DEGREES_MIN = SHARED.IS_RIPTIDE?-20:-20;
        public static final double ARM_ANGLE_DEGREES_MAX = SHARED.IS_RIPTIDE?180:180;

        public static final double CLOCK_ARM_POSITION_TOLERANCE = 2.0;

        public static final int ARM_CURRENT_LIMIT = 40;//amps

        public static final double ARM_MAX_ACCELERATION = 300;
        public static final double ARM_MAX_VELOCITY = 75; //previously 100

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

        public static final double WRIST_ANGLE_DEGREES_MIN = SHARED.IS_RIPTIDE?-45d:-45d;
        public static final double WRIST_ANGLE_DEGREES_MAX = SHARED.IS_RIPTIDE?220:220;

        public static final double WRIST_POSITION_TOLERANCE = 2.0;

        public static final int WRIST_CURRENT_LIMIT = 60;//amps

        public static final double WRIST_MAX_ACCELERATION = 300;
        public static final double WRIST_MAX_VELOCITY = 75; //used to be 100

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
        public static final Color OFF = new Color(0, 0, 0);
    }

    public final class DEEP_CLIMB{
        public static final double DEEP_CLIMB_START_POSITION = SHARED.IS_RIPTIDE? /*RIPTIDE: */-128: /*PERRY: */0;
    }

}
