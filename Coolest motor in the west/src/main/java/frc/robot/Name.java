package frc.robot;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
public class Name {
    public void setmotoroutput(double outputpercent){
double motormaxspeed = Coolestmotor.getSupplyVoltage().getValueAsDouble();
Coolestmotor.setControl(Coolestmotorvolts.withOutput(motormaxspeed*outputpercent));
    }
    private TalonFX Coolestmotor; 
     private VoltageOut Coolestmotorvolts = new VoltageOut(0);
    public Name ()  {


       
TalonFXConfiguration talonfxconfig = new TalonFXConfiguration();
Coolestmotor = new TalonFX(Constants.TEST_MOTOR_CAN_ID);
Coolestmotor.getConfigurator().apply(talonfxconfig);
Coolestmotor.setControl(Coolestmotorvolts.withOutput(0));

    }
}





