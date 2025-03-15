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
    private static Color color1 = new Color(0,0,0);
    private static int priority = 0;
    private static Color color2 = new Color(0,0,0);
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
        for(int i = 0; i < LEDS.LED_STRIP_LENGTH; i++){
            if(i%2==0){
                candle.setLEDs((int)(color1.red*255), (int)(color1.green*255), (int)(color1.blue*255), 0, i, 1);
            }
            else{
                candle.setLEDs((int)(color2.red*255), (int)(color2.green*255), (int)(color2.blue*255), 0, i, 1);
            }
        }
        priority = 0;
        color1 = new Color();
        color2 = new Color();
    }
    public static void setSolidColor(Color color, int priority){
        if(priority>priority){
            color2 = color1;
            color1 = color;
        }
    }
    // public static void setProgress(Color color, double progress, int priority){
    //     candle.setLEDs((int)color.red, (int)color.green, (int)color.blue, 0, 0, 20);
    // }
}
