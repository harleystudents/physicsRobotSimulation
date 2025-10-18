package utils.sim.hardwareSim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import utils.sim.MechanismState;

/**
 * A simulated DC motor controller that implements the MotorController interface.
 * This class can be controlled via voltage or current.
 */
public class SimulatedDCMotor implements SimMotorController {

    public enum ControlMode {
        VOLTAGE,
        CURRENT,
        POSITION
    }

    private ControlMode controlMode = ControlMode.VOLTAGE;
    private double commandedValue = 0.0;
    private boolean brakeMode = false;
    private double position = 0.0;
    private final ProfiledPIDController pidController;
    private final ArmFeedforward armFeedforward;
    private final String name;
    private boolean controlledThisCycle = false;
    double velocity = 0.0;

    public SimulatedDCMotor(String name) {
        this.pidController = new ProfiledPIDController(
            0.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(0.0, 0.0));
        this.armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        this.name = name;
    }

    @Override
    public void setConfig(TalonFXConfiguration config) {
        this.pidController.setPID(config.Slot0.kP, config.Slot0.kI, config.Slot0.kD);
        this.armFeedforward.setKa(config.Slot0.kA);
        this.armFeedforward.setKs(config.Slot0.kS);
        this.armFeedforward.setKg(config.Slot0.kG);
        this.pidController.setConstraints(
            new TrapezoidProfile.Constraints(config.MotionMagic.MotionMagicCruiseVelocity, config.MotionMagic.MotionMagicAcceleration));
    }

    /**
     * sets the current position of the motor in terms of the mechanism not the motor
     * notable an error might occur with unbounded hardlimits because radians wrap but it should work
     * @param the position of the mechanism
     */

    public void update(MechanismState state){
        this.position = state.getPosition();
        this.velocity = state.getVelocity();

    }

    /**
     * Sets the controller to voltage control mode and sets the voltage.
     * @param voltage The voltage to apply.
     */
    public void setVoltage(double voltage) {
        controlledThisCycle = true;
        this.controlMode = ControlMode.VOLTAGE;
        this.commandedValue = voltage;
        DogLog.log("Sim/" + name + "/SetVoltage", voltage);
    }
    @Override
    public double getPosition() {
        return position;
    }

    /**
     * Sets the controller to current control mode and sets the current.
     * @param current The current to apply.
     */
    public void setCurrent(double current) {
        this.controlledThisCycle = true;
        this.controlMode = ControlMode.CURRENT;
        this.commandedValue = current;
    }

    public void setBrakeMode(boolean brakeMode) {
        this.brakeMode = brakeMode;
    }
    /**
     * Position is in terms of the mechanism, not the motor (i.e. includes gearing)
     * Sets the controller to position control mode and sets the position.
     * @param position The position to go to in radians.
     */

    @Override
    public void goToPosition(double position) {
        this.controlledThisCycle = true;
        this.controlMode = ControlMode.POSITION;
        this.pidController.setGoal(position);
    }

    @Override
    public ControllerOutput run(double dt, double supplyVoltage) {
        if(!controlledThisCycle){
            // If the motor was not controlled this cycle, set voltage to 0
            this.commandedValue = 0.0;
            this.controlMode = ControlMode.VOLTAGE;
            controlledThisCycle = false;
            return new ControllerOutput(0.0, 0.0, false);
        }
        else if (controlMode == ControlMode.CURRENT) {
            // Return an object with the current value and a flag indicating to use current
            controlledThisCycle = false;
            return new ControllerOutput(0.0, this.commandedValue, true); 
        } else if (controlMode == ControlMode.POSITION) {
            controlledThisCycle = false;
            double voltage = this.pidController.calculate(this.position) + armFeedforward.calculate(this.position, this.velocity);
            DogLog.log("Sim/" + name + "/PositionError", this.pidController.getPositionError());
            DogLog.log("Sim/" + name + "/PositionSetpoint", this.pidController.getSetpoint().position);
            DogLog.log("Sim/" + name + "/PositionFeedforward", armFeedforward.calculate(this.position, this.velocity));
            voltage = voltage > 12.0 ? 12.0: voltage < -12.0 ? -12.0 : voltage; // Clamp to +/- 12V
            return new ControllerOutput(voltage, 0.0, false);
        } else {
            controlledThisCycle = false;
            // Return an object with the voltage value and a flag indicating to use voltage
            return new ControllerOutput(this.commandedValue, 0.0, false);
        }
    }

    @Override
    public boolean isBrakeMode() {
        return this.brakeMode;
    }
}