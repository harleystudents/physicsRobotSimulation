package utils.sim.hardwareSim;

import dev.doglog.DogLog;

/**
 * A simulated DC motor controller that implements the MotorController interface.
 * This class can be controlled via voltage or current.
 */
public class SimulatedDCMotor implements SimMotorController {

    public enum ControlMode {
        VOLTAGE,
        CURRENT
    }

    private ControlMode controlMode = ControlMode.VOLTAGE;
    private double commandedValue = 0.0;
    private boolean brakeMode = false;
    private double position = 0.0;

    public void setPosition(double position) {
        this.position = position;
    }

    /**
     * Sets the controller to voltage control mode and sets the voltage.
     * @param voltage The voltage to apply.
     */
    public void setVoltage(double voltage) {
        this.controlMode = ControlMode.VOLTAGE;
        this.commandedValue = voltage;
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
        this.controlMode = ControlMode.CURRENT;
        this.commandedValue = current;
    }

    public void setBrakeMode(boolean brakeMode) {
        this.brakeMode = brakeMode;
    }

    @Override
    public ControllerOutput run(double dt, double supplyVoltage) {
        if (controlMode == ControlMode.CURRENT) {
            // Return an object with the current value and a flag indicating to use current
            return new ControllerOutput(0.0, this.commandedValue, true); 
        } else {
            // Return an object with the voltage value and a flag indicating to use voltage
            return new ControllerOutput(this.commandedValue, 0.0, false);
        }
    }

    @Override
    public boolean isBrakeMode() {
        return this.brakeMode;
    }
}