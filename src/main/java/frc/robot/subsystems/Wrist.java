package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import utils.sim.hardwareSim.SimulatedDCMotor;
import utils.sim.mechanisms.ArmMechanism;

public class Wrist extends SubsystemBase {
    private final SimulatedDCMotor motor;
    private final ArmMechanism arm;
    private final Timer timer = new Timer();
    private WristState currentState = WristState.FORWARD;

    private enum WristState {
        FORWARD,
        BACKWARD,
        STOPPED
    }

    public Wrist() {
        motor = new SimulatedDCMotor();
        motor.setBrakeMode(true);
        arm = new ArmMechanism(
            DCMotor.getKrakenX60(1), // Motor
            motor,             // Motor Controller
            100.0,            // Gearing
            0.05,             // Moment of Inertia (kg m^2)
            -Math.PI / 2,      // Min Angle (radians)
            Math.PI / 2,       // Max Angle (radians)
            2.0,              // Weight (kg)
            0.1               // Center of Mass (meters from pivot)
        );
        timer.start();
    }

    public void runAtVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case FORWARD:
                runAtVoltage(6.0);
                if (timer.get() > 2.0) {
                    timer.reset();
                    currentState = WristState.BACKWARD;
                }
                break;
            case BACKWARD:
                runAtVoltage(-6.0);
                if (timer.get() > 2.0) {
                    timer.reset();
                    currentState = WristState.STOPPED;
                }
                break;
            case STOPPED:
                runAtVoltage(0.0);
                break;
        }
    }

    @Override
    public void simulationPeriodic() {
        arm.update(0.02);
    }
}