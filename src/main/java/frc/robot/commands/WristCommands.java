package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class WristCommands {
    public Command occilate(Wrist wrist){
        return Commands.sequence(
                Commands.runOnce(()->wrist.setVoltage(5), wrist).withTimeout(Second.of(3)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()->wrist.setVoltage(-5), wrist).withTimeout(Second.of(3)),
                Commands.waitSeconds(0.5));
    }
}
