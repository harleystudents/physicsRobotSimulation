package frc.robot.subsystems.wrist.mechanical;

import frc.robot.subsystems.Component;

public abstract class Arm extends Component{
    abstract public void setVoltage(double voltage);
    abstract public double getAngle();
    abstract public void periodic();
    
}
