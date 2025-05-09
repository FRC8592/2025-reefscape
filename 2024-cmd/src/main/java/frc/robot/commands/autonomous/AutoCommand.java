package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ScoreCoral;
import frc.robot.subsystems.scoring.Intake;
import frc.robot.subsystems.scoring.Scoring;

/**
 * Class to provide subsystems, convenient methods, and a constructor to autonomous commands
 */
public class AutoCommand extends WrapperCommand{
    protected static Swerve swerve;
    protected static Scoring scoring;
    protected static Intake intake;
    protected static LEDs leds;
    protected static ScoreCoral scoreCoral;

    private String autoName;
    
    public static void addSubsystems(Swerve swerve, Scoring scoring, Intake intake, LEDs leds, ScoreCoral scoreCoral){
        AutoCommand.swerve = swerve;
        AutoCommand.scoring = scoring;
        AutoCommand.intake = intake;
        AutoCommand.leds = leds;
        AutoCommand.scoreCoral = scoreCoral;
    }

    /**
     * If this is set, the odometry's known position will be set to this at the start of auto
     */
    protected Pose2d startPose = null;

    /**
     * Startup time saving if/when multiple copies of the same path are requested
     */
    private static HashMap<String, Trajectory> cachedChoreoTrajectories = new HashMap<String,Trajectory>();

    /**
     * Create an auto routine from the passed-in commands.
     *
     * @param commands as many commands as you want. Will
     * be run in sequence (one after the other).
     */
    protected AutoCommand(Command... commands) {
        super(Commands.sequence(commands));
        this.autoName = getClass().getSimpleName();
    }

    /**
     * {@link Commands#none()} as an {@link AutoCommand}
     */
    protected AutoCommand(){
        super(Commands.none());
        this.autoName = getClass().getSimpleName();
    }

    protected AutoCommand withAutoName(String name){
        this.autoName = name;
        return this;
    }

    protected String getAutoName(){
        return this.autoName;
    }

    /**
     * Get a choreo trajectory by name as a WPILib trajectory.
     *
     * @param name the name of the .traj file; this shouldn't contain the path or
     * the filename extension
     *
     * @return The trajectory converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .traj file
     * located in the {@code choreo} folder in the {@code deploy} folder
     */
    protected static final Trajectory getChoreoTrajectory(String name){
        return getChoreoTrajectory(name, -1);
    }
    protected static final Trajectory getChoreoTrajectory(String name, int splitIndex){
        System.out.println("Grabbed path "+name);
        if(cachedChoreoTrajectories.containsKey(name)){
            return cachedChoreoTrajectories.get(name);
        }
        else{
            try{
                Trajectory wpilibTrajectory;
                if(splitIndex == -1){
                    wpilibTrajectory = fromChoreoPath((choreo.trajectory.Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get());
                }
                else{
                   wpilibTrajectory = fromChoreoPath((choreo.trajectory.Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get().getSplit(splitIndex).get());
                }
            //    Trajectory wpilibTrajectory = new Trajectory();

                cachedChoreoTrajectories.put(name, wpilibTrajectory);
                return wpilibTrajectory;
            }
            catch(Exception e){
                throw new RuntimeException(e);
            }
            // return new Trajectory();
        }
    }

    /**
     * Set the start pose of this auto to the first pose of a Choreo path.
     *
     * @param name the name of the Choreo path to get the start pose from
     */
    protected void setStartStateFromChoreoTrajectory(String name){
        if(!cachedChoreoTrajectories.containsKey(name)){
            getChoreoTrajectory(name); // Adds the path to the cached trajectory map
        }
        this.startPose = cachedChoreoTrajectories.get(name).getInitialPose();
    }

    /**
     * Convert a PathPlanner path into a WPILib trajectory
     *
     * @param path the PathPlannerPath to convert
     * @return the path converted to a WPILib trajectory
     */
    private static Trajectory fromChoreoPath(choreo.trajectory.Trajectory<SwerveSample> path){
        List<SwerveSample> choreoSamples = path.samples();
        ArrayList<State> wpilibStates = new ArrayList<>();

        // Convert all the PathPlanner states to WPILib trajectory states and add
        // them to the wpilibStates ArrayList
        for (SwerveSample choreoSample : choreoSamples) {
            double metersPerSecond = Math.sqrt(Math.pow(choreoSample.vx, 2)+Math.pow(choreoSample.vy, 2));

            State wpilibState = new State(
                choreoSample.t,
                metersPerSecond,
                Math.sqrt(Math.pow(choreoSample.ax, 2)+Math.pow(choreoSample.ay, 2)),
                choreoSample.getPose(),
                choreoSample.omega/metersPerSecond
            );
            wpilibStates.add(wpilibState);
        }
        return new Trajectory(wpilibStates);
    }
}
