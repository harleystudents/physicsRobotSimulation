package frc.robot.subsystems.wrist.mechanical;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.wrist.WristVisualizer;
import utils.sim.HardLimits;
import utils.sim.hardwareSim.SimulatedDCMotor;
import utils.sim.mechanisms.ArmMechanism;

public class ArmSim extends Arm {
    private final SimulatedDCMotor motor;
    private final ArmMechanism arm;
    private final Timer timer = new Timer();
    private double counter = 0.0;
    private final WristVisualizer visualizer = new WristVisualizer(true);

    public ArmSim() {
        motor = new SimulatedDCMotor();
        motor.setBrakeMode(true);
        // motor.setBrakeMode(true);
        arm = new ArmMechanism(
            DCMotor.getKrakenX60(1), // Motor
            motor,             // Motor Controller
            20,            // Gearing
            0.2,             // Moment of Inertia (kg m^2)
            new HardLimits(-Math.PI/2, Math.PI/2), // Min/Max Angle (radians)
            10.0,              // Weight (kg)
            0.1               // Center of Mass (meters from pivot)
        );
        timer.start();
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Voltage", voltage);
    }
    public double getAngle() {
        return arm.getState().getPosition();
    }

    @Override
    public void periodic() {
        double dt = Math.min(timer.get(), 0.05);
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/dt", dt);
        arm.update(dt);
        timer.reset();
        visualizer.update(arm.getState().getPosition()*180/Math.PI, arm.getTorques());
        
    }
}
