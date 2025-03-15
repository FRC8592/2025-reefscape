package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class LEDs {
    private static CANdle candle;
    private static boolean hasCoral; 
    private static boolean coralMode;
    private static double progressBar;

    public static void init(){
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle = new CANdle(43,"DriveTrain");
        candle.configAllSettings(configAll, 100);
    }

    public static void writeLEDs(){
        displayHasCoralLEDs();
        displayModeLEDs();
    }

    public static void displayModeLEDs(){
        if(coralMode){
            candle.setLEDs((int)(LEDS.ORANGE.red*255),(int)(LEDS.ORANGE.green*255),(int)(LEDS.ORANGE.blue*255),0,0, LEDS.LED_STRIP_LENGTH/2);
        }
        else{
            candle.setLEDs((int)(LEDS.TEAL.red*255),(int)(LEDS.TEAL.green*255),(int)(LEDS.TEAL.blue*255),0,0, LEDS.LED_STRIP_LENGTH/2);  
        }

    }

    public static void displayHasCoralLEDs(){
        if(hasCoral){
            candle.setLEDs((int)(LEDS.WHITE.red*255),(int)(LEDS.WHITE.green*255),(int)(LEDS.WHITE.blue*255),0,LEDS.LED_STRIP_LENGTH/2, LEDS.LED_STRIP_LENGTH);  
        }
        else{
            candle.setLEDs((int)(LEDS.OFF.red*255),(int)(LEDS.OFF.green*255),(int)(LEDS.OFF.blue*255),0,LEDS.LED_STRIP_LENGTH/2, LEDS.LED_STRIP_LENGTH);  
        }
    }

    public static void displayProgressBarLEDs(){
        
        candle.setLEDs((int)(LEDS.GREEN.red*255),(int)(LEDS.GREEN.green*255),(int)(LEDS.GREEN.blue*255),0,0,(int)(LEDS.LED_STRIP_LENGTH * progressBar));  
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
}
