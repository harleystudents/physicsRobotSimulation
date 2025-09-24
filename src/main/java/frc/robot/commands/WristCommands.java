package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class WristCommands {
    public static Command occilate(Wrist wrist){
        return Commands.sequence(
                        Commands.runOnce(()->DogLog.log("Robot/Commands/Ocilation", true)).withTimeout(.05),
                        runAtVoltage(wrist, 5.0),
                        runAtVoltage(wrist, -5.0),
                        Commands.runOnce(() -> DogLog.log("Robot/Commands/Ocilation", false)).withTimeout(.05)
                        ).repeatedly();
    }
    public static Command runAtVoltage(Wrist wrist, double voltage){
        return Commands.runOnce((() -> wrist.setVoltage(voltage)), wrist).withTimeout(4.0).andThen(Commands.runOnce(()->wrist.setVoltage(0.0), wrist)).withTimeout(2.0).withName("Run Wrist at " + voltage + "V");
    }
}
