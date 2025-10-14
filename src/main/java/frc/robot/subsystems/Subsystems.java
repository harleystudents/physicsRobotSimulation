package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.elevator.Elevator;


public class Subsystems {

  public final Wrist wrist;
  public final Elevator elevator;

  public Subsystems(Wrist wrist, Elevator elevator) {
    this.wrist = wrist;
    this.elevator = elevator;
    // wrist.setDefaultCommand(WristCommands.runAtVoltage(wrist, 2.0));

    ExclusiveSubsystem[] lockedResources = {wrist, elevator};
    SharedSubsystem[] locklessResources = {};

    CommandScheduler.getInstance().registerSubsystem(lockedResources);
    for (SharedSubsystem subsystem : locklessResources) {
      CommandScheduler.getInstance()
          .registerSubsystem(
              new Subsystem() {
                @Override
                public void periodic() {
                  subsystem.periodic();
                }

                @Override
                public String getName() {
                  return subsystem.getName();
                }
              });
    }
  }

  /**
   * @see Subsystem
   */
  public static interface ExclusiveSubsystem extends Subsystem {}

  /** A subsystem that does not act as a locking resource when using commands */
  public static interface SharedSubsystem {
    default void periodic() {}

    default void simulationPeriodic() {}

    default String getName() {
      return this.getClass().getSimpleName();
    }
  }
}

