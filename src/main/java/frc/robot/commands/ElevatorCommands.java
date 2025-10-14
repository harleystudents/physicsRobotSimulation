package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
  public static Command runAtVoltage(Elevator elevator, double voltage) {
    return elevator
        .run(() -> elevator.setVoltage(voltage))
        .withName("SetElevatorVoltage: " + voltage);
  }

  public static Command goToHeight(Elevator elevator, double heightMeters) {
    return elevator
        .run(() -> elevator.setHeight(heightMeters))
        .withName("GoToHeight: " + heightMeters);
  }
}
