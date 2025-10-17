package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystems.ExclusiveSubsystem;
import frc.robot.subsystems.elevator.mechanical.ElevatorBase;
import frc.robot.subsystems.wrist.mechanical.Arm;
import frc.robot.subsystems.wrist.mechanical.ArmSim;

public class Elevator implements ExclusiveSubsystem{
    
    private final ElevatorBase elevatorBase;
    
    public Elevator(){
        if (Robot.isSimulation()){
            elevatorBase = new frc.robot.subsystems.elevator.mechanical.ElevatorSim();
        } else {
            DriverStation.reportError("No Elevator implementation found", false);
            elevatorBase = null;
        }

    }

    public void setVoltage(double voltage){
        DogLog.log("Robot/Subsystems/Elevator/IN Elevator.Java VOLTAGE SET TO", voltage);
        elevatorBase.setVoltage(voltage);
    }

    public void setHeight(double heightMeters){
        DogLog.log("Robot/Subsystems/Elevator/IN Elevator.Java ANGLE SET TO", heightMeters);
        elevatorBase.setHeight(heightMeters);
    }
    
    public double getHeight(){
        return elevatorBase.getHeight();
    }

    @Override
    public void periodic(){
        elevatorBase.periodic();
        
    }
}
