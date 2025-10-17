package utils.sim.mechanisms;

import java.util.LinkedHashMap;
import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import utils.debug.DebugUtils;
import utils.sim.HardLimits;
import utils.sim.MechanismState;
import utils.sim.SimFrameWork;
import utils.sim.Torques;
import utils.sim.hardwareSim.SimMotorController;

/**
 * A simple physics simulator for a single-stage elevator driven by a drum.
 *
 * <p>The simulator models the carriage as a point mass that is driven by a
 * motor through a gearbox and a drum of fixed radius. Forces acting on the
 * carriage include the motor force, gravity and an optional viscous braking
 * term that is applied when the motor controller is in brake mode.</p>
 */
public class ElevatorMechanism extends SimFrameWork {

    private final MechanismState state = new MechanismState(0.0, 0.0, 0.0);
    private final Torques torques = new Torques(0.0, 0.0, 0.0);

    private final DCMotor motor;
    private final SimMotorController controller;
    private final double gearing;
    private final double drumRadius;
    private final double carriageMass;
    private final HardLimits limits;
    private final double brakeCoefficient;

    private double motorForce = 0.0;
    private double gravityForce = 0.0;
    private double brakingForce = 0.0;

    /** Acceleration due to gravity (m/s^2). */
    private static final double GRAVITY = -9.80665;

    /** Maximum magnitude of linear acceleration allowed for the carriage. */
    private final double maxAcceleration;

    /**
     * Creates a new {@link ElevatorMechanism} instance.
     *
     * @param motor The motor driving the elevator.
     * @param controller The simulated motor controller.
     * @param gearing The gearbox ratio expressed as output speed divided by input speed.
     * @param drumRadius The radius of the elevator drum in meters.
     * @param carriageMass The mass of the elevator carriage in kilograms.
     * @param limits Hard limits that bound the carriage travel.
     * @param brakeCoefficient The viscous braking coefficient (N·s/m) applied when brake mode is enabled.
     * @param maxAcceleration Maximum magnitude of carriage acceleration (m/s^2). Use {@code Double.POSITIVE_INFINITY}
     *                        to disable clamping.
     */
    public ElevatorMechanism(
            DCMotor motor,
            SimMotorController controller,
            double gearing,
            double drumRadius,
            double carriageMass,
            HardLimits limits,
            double brakeCoefficient,
            double maxAcceleration) {
        this.motor = motor;
        this.controller = controller;
        this.gearing = gearing;
        this.drumRadius = drumRadius;
        this.carriageMass = carriageMass;
        this.limits = limits;
        this.brakeCoefficient = brakeCoefficient;
        double sanitizedMaxAcceleration = Math.abs(maxAcceleration);
        if (Double.isNaN(sanitizedMaxAcceleration)) {
            sanitizedMaxAcceleration = Double.POSITIVE_INFINITY;
        }
        this.maxAcceleration = sanitizedMaxAcceleration;
    }

    @Override
    public MechanismState getState() {
        DogLog.log("Sim/Elevator/Position", state.getPosition());
        DogLog.log("Sim/Elevator/Velocity", state.getVelocity());
        DogLog.log("Sim/Elevator/Acceleration", state.getAcceleration());
        return state;
    }

    @Override
    public Torques getTorques() {
        torques.mTorque = motorForce * drumRadius;
        torques.gTorque = gravityForce * drumRadius;
        torques.bTorque = brakingForce * drumRadius;
        return torques;
    }

    @Override
    public void update(double dt) {
        var output = controller.run(dt, 12.0);
        DogLog.log("Sim/Elevator/Voltage", output.voltage());
        DogLog.log("Sim/Elevator/Current", output.current());

        double carriageVelocity = state.getVelocity();
        double drumAngularVelocity = carriageVelocity / drumRadius;
        double motorVelocity = drumAngularVelocity * gearing;

        double motorTorque = 0.0;
        if (output.useCurrent()) {
            motorTorque = motor.getTorque(output.current()) * gearing;
        } else {
            double appliedVoltage = output.voltage();
            if (appliedVoltage != 0.0) {
                motorTorque = motor.getTorque(motor.getCurrent(motorVelocity, appliedVoltage)) * gearing;
            }
        }

        motorForce = motorTorque / drumRadius;

        if (controller.isBrakeMode() && carriageVelocity != 0.0) {
            brakingForce = -Math.signum(carriageVelocity) * brakeCoefficient * Math.abs(carriageVelocity);
        } else {
            brakingForce = 0.0;
        }

        gravityForce = carriageMass * GRAVITY;

        double totalForce = motorForce + brakingForce + gravityForce;
        double unclampedAcceleration = totalForce / carriageMass;

        boolean accelerationClamped = false;
        if (Double.isFinite(maxAcceleration) && maxAcceleration > 0.0) {
            double maxTotalForce = maxAcceleration * carriageMass;
            double clampedTotalForce = MathUtil.clamp(totalForce, -maxTotalForce, maxTotalForce);
            if (clampedTotalForce != totalForce) {
                motorForce = clampedTotalForce - (brakingForce + gravityForce);
                totalForce = clampedTotalForce;
                accelerationClamped = true;
            }
        }
        DogLog.log("Sim/Elevator/AccelerationClamped", accelerationClamped);
        DogLog.log("Sim/Elevator/AccelerationLimit", maxAcceleration);
        DogLog.log("Sim/Elevator/UnclampedAcceleration", unclampedAcceleration);

        state.setAcceleration(totalForce / carriageMass);
        state.setVelocity(state.getVelocity() + state.getAcceleration() * dt);
        state.setPosition(state.getPosition() + state.getVelocity() * dt);

        if (!limits.isUnbounded()) {
            if (state.getPosition() > limits.getMax()) {
                state.setPosition(limits.getMax());
                state.setVelocity(0.0);
                DogLog.log("Sim/Elevator/Is At Hard Lower Limit", false);
                DogLog.log("Sim/Elevator/Is At Hard Upper Limit", true);
            } else if (state.getPosition() < limits.getMin()) {
                state.setPosition(limits.getMin());
                state.setVelocity(0.0);
                DogLog.log("Sim/Elevator/Is At Hard Lower Limit", true);
                DogLog.log("Sim/Elevator/Is At Hard Upper Limit", false);
            } else {
                DogLog.log("Sim/Elevator/Is At Hard Lower Limit", false);
                DogLog.log("Sim/Elevator/Is At Hard Upper Limit", false);
            }
        }
        Map<String, Object> vars = new LinkedHashMap<>();
        vars.put("motorVelocity", motorVelocity);
        vars.put("motor torque", motorTorque);
        vars.put("motorForce", motorForce);
        vars.put("brakingForce", brakingForce);
        vars.put("gravityForce", gravityForce);
        vars.put("state accel", state.getAcceleration());
        vars.put("state vel", state.getVelocity());
        vars.put("state pos", state.getPosition());



        DebugUtils.dumpVars(vars);

        controller.update(state);
    }
}
