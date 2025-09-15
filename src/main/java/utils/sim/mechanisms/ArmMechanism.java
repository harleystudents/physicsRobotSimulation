package utils.sim.mechanisms;

import edu.wpi.first.math.system.plant.DCMotor;
import utils.sim.SimFrameWork;
import utils.sim.hardwareSim.SimMotorController;

public class ArmMechanism extends SimFrameWork{
    public static class MechanismState {
        public double position;
        public double velocity;
        public double acceleration;
    private MechanismState state = new MechanismState(0.0, 0.0, 0.0);
    private DCMotor motor;
    private double gearing;
    private double momentOfInertia;
    private double minAngle;
    private double maxAngle;
    private double weight;
    private double centerOfMass;
    private SimMotorController controller;
    /** 
     * Creates a new ArmMechanism instance.
     *
     * @param motor The motor used to actuate the arm.
     * @param gearing The gear ratio of the arm (output speed / input speed).
     * @param momentOfInertia The moment of inertia of the arm (kg m^2).
     * @param minAngle The minimum angle of the arm (radians).
     * @param maxAngle The maximum angle of the arm (radians).
     * @param weight The weight of the arm (kg).
     * @param centerOfMass The center of mass of the arm (meters from pivot).
     */
    public ArmMechanism(
        DCMotor motor,
        SimMotorController controller,
        double gearing,
        double momentOfInertia,
        double minAngle,
        double maxAngle,
        double weight,
        double centerOfMass
    ) {
        this.motor = motor;
        this.gearing = gearing;
        this.momentOfInertia = momentOfInertia;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.weight = weight;
        this.centerOfMass = centerOfMass;
        this.controller = controller;
    }

    @Override
    public void update(double dt) {
        // For now, assume a fixed 12V supply
        var output = this.controller.run(dt, 12.0);
        
        double motorTorque;
        double mechanismVelocity = state.velocity() * this.gearing;

        // 2. Calculate motor torque based on the controller output type
        if (output.useCurrent()) {
            // Controller is providing a target current
            motorTorque = this.motor.getTorque(output.current());
        } else {
            // Controller is providing a target voltage
            double appliedVoltage = output.voltage();

            // If brake mode is on and commanded voltage is near zero, apply strong resistance
            if (this.controller.isBrakeMode() && Math.abs(appliedVoltage) < 1e-2) {
                this.state.velocity *= 0.1; 
                appliedVoltage = 0;
            }
            
            motorTorque = this.motor.getTorque(this.motor.getCurrent(mechanismVelocity, appliedVoltage));
        }
        
        // 3. Calculate physics
        final double g = 9.8;
        double gravityTorque = this.centerOfMass * this.weight * g * Math.cos(state.position());
        double totalTorque = (motorTorque * this.gearing) - gravityTorque;

        // 4. Update state via Euler integration
        this.state.acceleration = totalTorque / this.momentOfInertia;
        this.state.velocity += this.state.acceleration * dt;
        this.state.position += this.state.velocity * dt;

        // 5. Apply constraints
        if (this.state.position > this.maxAngle) {
            this.state.position = this.maxAngle;
            this.state.velocity = 0;
        } else if (this.state.position < this.minAngle) {
            this.state.position = this.minAngle;
            this.state.velocity = 0;
        }
    }
}
