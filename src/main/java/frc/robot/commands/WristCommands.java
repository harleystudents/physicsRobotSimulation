package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class WristCommands {
    public static Command occilate(Wrist wrist){
        return Commands.deadline(
                Commands.sequence(
                        Commands.run(() -> wrist.setVoltage(5), wrist).withTimeout(1),
                        Commands.runOnce(() -> wrist.setVoltage(0)).withTimeout(.1), // Stop the motor
                        Commands.waitSeconds(0.5),
                        Commands.run(() -> wrist.setVoltage(-5), wrist).withTimeout(3),
                        Commands.runOnce(() -> wrist.setVoltage(0)).withTimeout(.1) // Stop the motor
                        ).withName("OCCILATING"));
    }
    public static Command runAtVoltage(Wrist wrist, double voltage){
        return Commands.runOnce((() -> wrist.setVoltage(voltage)), wrist).withTimeout(4.0).andThen(Commands.runOnce(()->wrist.setVoltage(0.0), wrist)).withName("Run Wrist at " + voltage + "V");
    }
}
