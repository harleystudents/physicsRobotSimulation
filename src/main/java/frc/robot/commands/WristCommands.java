package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class WristCommands {
    public static Command runAtVoltage(Wrist wrist, double voltage){
        return wrist.run((() -> wrist.setVoltage(voltage))).withName("SetWristVoltage: "+voltage);
    }
}
