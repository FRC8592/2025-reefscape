package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class LEDs {
    private static CANdle candle;
    private static boolean hasCoral; 
    private static boolean coralMode;
    private static double progressBar = -1;
    private static int tagCount;
    private static boolean deepclimb;
    private static boolean useRainbow;
    private static Timer timer = new Timer(); 
    private static RainbowAnimation rainbow = new RainbowAnimation(1,0.5,LEDS.FULL_LED_COUNT) ;

    public static void init(){
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle = new CANdle(46);
        candle.configAllSettings(configAll, 100);
        timer.start();
    }

    public static void writeLEDs(){
        if(DriverStation.isDisabled()){
            displayHasTagsLEDs();
        }

        else{
            if (useRainbow){
                displayRaindow();
            }
            else if(deepclimb){
                displayDeepClimb();
            }
            else if(progressBar != -1){
                displayProgressBarLEDs();
            }
            else if(hasCoral){
                displayHasCoralLEDs();
            }
            else{
                displayModeLEDs();
            }

        }
    }

    public static void displayModeLEDs(){
        if(coralMode){
            candle.setLEDs((int)(LEDS.WHITE.red*255),(int)(LEDS.WHITE.green*255),(int)(LEDS.WHITE.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);
        }
        else{
            candle.setLEDs((int)(LEDS.TEAL.red*255),(int)(LEDS.TEAL.green*255),(int)(LEDS.TEAL.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
        }

    }

    public static void displayHasCoralLEDs(){
        if(hasCoral){
            if((int)(timer.get()*3) % 2 == 0){
                candle.setLEDs((int)(LEDS.WHITE.red*255),(int)(LEDS.WHITE.green*255),(int)(LEDS.WHITE.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
            }
            else{
                candle.setLEDs((int)(LEDS.OFF.red*255),(int)(LEDS.OFF.green*255),(int)(LEDS.OFF.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
            }
        }
    }

    public static void displayHasTagsLEDs(){
        if(tagCount>=2){
            candle.setLEDs((int)(LEDS.GREEN.red*255),(int)(LEDS.GREEN.green*255),(int)(LEDS.GREEN.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
        }
        else if(tagCount==1){
            candle.setLEDs((int)(LEDS.YELLOW.red*255),(int)(LEDS.YELLOW.green*255),(int)(LEDS.YELLOW.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
        }
        else if(tagCount==0){
            candle.setLEDs((int)(LEDS.RED.red*255),(int)(LEDS.RED.green*255),(int)(LEDS.RED.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
        }

        else if(tagCount ==-1){
            if((int)(timer.get()*3) % 2 == 0){
                candle.setLEDs((int)(LEDS.RED.red*255),(int)(LEDS.RED.green*255),(int)(LEDS.RED.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
            }

            else{
                candle.setLEDs((int)(LEDS.OFF.red*255),(int)(LEDS.OFF.green*255),(int)(LEDS.OFF.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
            }
        }

    
    }
    public static void displayDeepClimb(){
        candle.setLEDs((int)(LEDS.PURPLE.red*255),(int)(LEDS.PURPLE.green*255),(int)(LEDS.PURPLE.blue*255),0,LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT);  
    }

    public static void displayProgressBarLEDs(){
        
        candle.setLEDs((int)(LEDS.GREEN.red*255),(int)(LEDS.GREEN.green*255),(int)(LEDS.GREEN.blue*255),0,0,(int)(LEDS.FULL_LED_COUNT * progressBar));  
        candle.setLEDs((int)(LEDS.OFF.red*255),(int)(LEDS.OFF.green*255),(int)(LEDS.OFF.blue*255),0,(int)(LEDS.FULL_LED_COUNT * progressBar),LEDS.FULL_LED_COUNT);  
    }

    public static void displayRaindow(){
        candle.animate(rainbow);
    }

    public static void setHasCoral(boolean robotHasCoral){
        hasCoral = robotHasCoral;
    }

    public static void setCoralMode(boolean isCoralMode){
        coralMode = isCoralMode;
    }

    public static void setProgressBar(double progress){
        progressBar = progress;
    }

    public static void setHasTags(int cameraTagCount){
        tagCount = cameraTagCount;
    }

    public static void setDeepClimb(boolean isDeepClimb){
        deepclimb = isDeepClimb;
    }

    public static void setRainbow(boolean isRainbowAnimation){
        useRainbow = false; // set equal to isRainbowAnimation when ready to test
    }
    
    
}
    
