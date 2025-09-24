package frc.robot.subsystems.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Subsystems.ExclusiveSubsystem;
import frc.robot.subsystems.wrist.mechanical.Arm;
import frc.robot.subsystems.wrist.mechanical.ArmSim;

public class Wrist implements ExclusiveSubsystem{
    private final Arm wrist;
    private final WristVisualizer visualizer = new WristVisualizer();
    
    public Wrist(){
        if (true){
            wrist = new ArmSim();
        } else {
            DriverStation.reportError("No wrist implementation found", false);
            wrist = null;
        }

    }

    public void setVoltage(double voltage){
        DogLog.log("Robot/Subsystems/Wrist/IN WRIST.Java VOLTAGE SET TO", voltage);
        wrist.setVoltage(voltage);
    }
    public double getAngle(){
        return wrist.getAngle();
    }
    @Override
    public void periodic(){
        wrist.periodic();
        visualizer.update(Math.toDegrees(getAngle()));
        
    }


}
