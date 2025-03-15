package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private CANdle candle;
    public LEDs(){
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.On;
        configAll.v5Enabled = false;
        candle = new CANdle(43);
        candle.configAllSettings(configAll, 500);
    }
    public void setColor(Color color){
        candle.setLEDs((int)(255*color.red), (int)(255*color.green), (int)(255*color.blue), 0, 0, 20);
    }
}