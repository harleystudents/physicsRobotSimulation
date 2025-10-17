package frc.robot.subsystems.elevator.mechanical;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.elevator.ElevatorVisualizer;
import utils.sim.HardLimits;
import utils.sim.hardwareSim.SimulatedDCMotor;
import utils.sim.mechanisms.ElevatorMechanism;

/** Simulation-backed elevator mechanism implementation. */
public class ElevatorSim extends ElevatorBase {
  private final SimulatedDCMotor motor;
  private final ElevatorMechanism elevator;
  private final Timer timer = new Timer();
  private final ElevatorVisualizer visualizer = new ElevatorVisualizer(true);

  private boolean positionControlled = false;
  private double desiredHeightMeters = 0.0;

  public ElevatorSim() {
    motor = new SimulatedDCMotor("elevator sim");
    motor.setBrakeMode(true);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 20.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.4;
    config.Slot0.kG = 0.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 1.5;
    motor.setConfig(config);

    elevator =
        new ElevatorMechanism(
            DCMotor.getKrakenX60(2), // type of motor
            motor,
            12.0, // Motor to mechanism gearing
            0.025, //meters
            6.0, // kg
            new HardLimits(0.0, 1.5), // meters
            30.0);

    timer.start();
  }

  @Override
  public void setVoltage(double voltage) {
    positionControlled = false;
    motor.setVoltage(voltage);
    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/Voltage", voltage);
  }

  @Override
  public void setHeight(double heightMeters) {
    positionControlled = true;
    desiredHeightMeters = heightMeters;
    motor.goToPosition(heightMeters);
    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/DesiredHeight", heightMeters);
  }

  @Override
  public double getHeight() {
    return elevator.getState().getPosition();
  }

  @Override
  public void periodic() {
    double dt = Math.min(timer.get(), 0.05);
    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/dt", dt);
    elevator.update(dt);
    timer.reset();
    
    visualizer.update(elevator.getState().getPosition(), desiredHeightMeters, elevator.getTorques(), positionControlled);

    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/Height", getHeight());
    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/ClosedLoop", positionControlled);
    DogLog.log("Robot/Subsystems/Elevator/ElevatorSim/Setpoint", desiredHeightMeters);
  }
}
