package frc.robot.subsystems.wrist.mechanical;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import utils.sim.hardwareSim.SimulatedDCMotor;
import utils.sim.mechanisms.ArmMechanism;

public class ArmSim extends Arm {
    private final SimulatedDCMotor motor;
    private final ArmMechanism arm;
    private final Timer timer = new Timer();
    private WristState currentState = WristState.FORWARD;

    private enum WristState {
        FORWARD,
        BACKWARD,
        STOPPED
    }

    public ArmSim() {
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

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Voltage", voltage);
    }
    public double getAngle() {
        return arm.getState().getPosition();
    }

    @Override
    public void periodic() {
        arm.update(timer.get());
        timer.reset();
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Position", arm.getState().getPosition());
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Velocity", arm.getState().getVelocity());
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Acceleration", arm.getState().getAcceleration());
    }
}
