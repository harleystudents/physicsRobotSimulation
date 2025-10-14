package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystems.ExclusiveSubsystem;
import frc.robot.subsystems.elevator.mechanical.ElevatorBase;
import frc.robot.subsystems.elevator.mechanical.ElevatorSim;

public class Elevator implements ExclusiveSubsystem {
  private final ElevatorBase elevator;

  public Elevator() {
    if (Robot.isSimulation()) {
      elevator = new ElevatorSim();
    } else {
      DriverStation.reportError("No elevator implementation found", false);
      elevator = null;
    }
  }

  public void setVoltage(double voltage) {
    DogLog.log("Robot/Subsystems/Elevator/VoltageCommand", voltage);
    if (elevator != null) {
      elevator.setVoltage(voltage);
    }
  }

  public void setHeight(double heightMeters) {
    DogLog.log("Robot/Subsystems/Elevator/HeightCommand", heightMeters);
    if (elevator != null) {
      elevator.setHeight(heightMeters);
    }
  }

  public double getHeight() {
    return elevator != null ? elevator.getHeight() : 0.0;
  }

  @Override
  public void periodic() {
    if (elevator != null) {
      elevator.periodic();
    }
  }
}
