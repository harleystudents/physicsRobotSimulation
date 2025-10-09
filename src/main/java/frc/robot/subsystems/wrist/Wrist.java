package frc.robot.subsystems.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystems.ExclusiveSubsystem;
import frc.robot.subsystems.wrist.mechanical.Arm;
import frc.robot.subsystems.wrist.mechanical.ArmSim;

public class Wrist implements ExclusiveSubsystem{
    private final Arm wrist;
    
    public Wrist(){
        if (Robot.isSimulation()){
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

    public void setPosition(double angle){
        DogLog.log("Robot/Subsystems/Wrist/IN WRIST.Java ANGLE SET TO", angle);
        wrist.setPosition(angle);
    }
    public double getAngle(){
        return wrist.getAngle();
    }
    @Override
    public void periodic(){
        wrist.periodic();
        
    }


}
