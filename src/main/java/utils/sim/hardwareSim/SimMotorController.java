package utils.sim.hardwareSim;

/**
 * An interface to represent a motor controller in the simulation.
 */
public interface SimMotorController {
    
    /**
     * A data object holding the output of a motor controller.
     * @param voltage The voltage output.
     * @param current The current output.
     * @param useCurrent A flag to indicate if the current value should be used (true)
     *                   or if the voltage value should be used (false).
     */
    public record ControllerOutput(double voltage, double current, boolean useCurrent) {}

    /**
     * Runs the simulation for the motor.
     *
     * @param dt the time step for the simulation.
     * @param supplyVoltage the supply voltage from the battery.
     * @param state the current state of the mechanism (e.g., position, velocity).
     * @return the output of the controller.
     */
    ControllerOutput run(double dt, double supplyVoltage);

    /**
     * Returns whether the brake mode is enabled on the motor controller.
     *
     * @return true if brake mode is enabled, false otherwise.
     */
    boolean isBrakeMode();

    /**
     * Returns the current position of the motor (if applicable).
     * @return The current position.
     */
    double getPosition();

    /**
     * sets the current position of the motor (if applicable).
     * @return void
     */
    void setPosition(double position);

    /**
     * A factory method for a motor controller that does nothing.
     * @return A new no-op motor controller.
     */
    public static SimMotorController none() {
        return new SimMotorController() {
            @Override
            public ControllerOutput run(double dt, double supplyVoltage) {
                // Returns a zero-voltage output by default
                return new ControllerOutput(0.0, 0.0, false);
            }
            @Override
            public void setPosition(double position) {
                // No-op
            }

            @Override
            public boolean isBrakeMode() {
                return false;
            }

            @Override
            public double getPosition() {
                return 0.0;
            }
        };
    }
}
