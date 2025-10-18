package frc.robot.subsystems.wrist.mechanical;

import java.util.function.BiPredicate;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.google.errorprone.annotations.Var;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.wrist.WristVisualizer;
import utils.sim.HardLimits;

import utils.sim.hardwareSim.SimulatedDCMotor;
import utils.sim.mechanisms.ArmMechanism;

public class ArmSim extends Arm {
    private final SimulatedDCMotor motor;
    private final ArmMechanism arm;
    private boolean positionControlled = false;
    private double desiredAngleRads = 0.0;
    private final Timer timer = new Timer();
    private final WristVisualizer visualizer = new WristVisualizer(true);

    public ArmSim() {
        motor = new SimulatedDCMotor("Arm Motor");
        motor.setBrakeMode(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.1;
        config.Slot0.kD = 0.1;
        config.Slot0.kG = .8;
        config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        config.MotionMagic.MotionMagicAcceleration = 1.0;
        config.Feedback.SensorToMechanismRatio = 20.0;

        motor.setConfig(config);

        // motor.setBrakeMode(true);
        arm = new ArmMechanism(
            DCMotor.getKrakenX60(1), // Motor
            motor,             // Motor Controller
            20,            // Gearing
            0.2,             // Moment of Inertia (kg m^2)
            new HardLimits(true), // Min/Max Angle (radians)
            10.0,              // Weight (kg)
            0.1               // Center of Mass (meters from pivot)
        );
        timer.start();
    }

    public void setVoltage(double voltage) {
        positionControlled = false;
        motor.setVoltage(voltage);
        DogLog.log("Robot/Subsystems/Wrist/ArmSim/Voltage", voltage);
    }

    public void setPosition(double angleRads) {
        positionControlled = true;
        desiredAngleRads = angleRads;
        motor.goToPosition(angleRads);
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
        visualizer.update(arm.getState().getPosition()*180/Math.PI, desiredAngleRads*180/Math.PI, positionControlled, arm.getTorques());
        
    }
}
