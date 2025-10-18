package frc.robot.subsystems.elevator.mechanical;

import frc.robot.subsystems.Component;

/** Base abstraction for elevator mechanism implementations. */
public abstract class ElevatorBase extends Component {
  public abstract void setVoltage(double voltage);

  public abstract void setHeight(double heightMeters);

  public abstract double getHeight();

  
}
