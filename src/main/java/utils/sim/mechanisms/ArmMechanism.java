package utils.sim.mechanisms;



import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import utils.sim.HardLimits;
import utils.sim.MechanismState;
import utils.sim.SimFrameWork;
import utils.sim.Torques;
import utils.sim.hardwareSim.SimMotorController;



public class ArmMechanism extends SimFrameWork{

    private MechanismState state = new MechanismState(Math.PI/4, 0.0, 0.0);
    private Torques torques = new Torques(0.0, 0.0, 0.0);
    private double motorTorque = 0.0;
    private double brakingTorque = 0.0;
    private double gravityTorque = 0.0;

    private DCMotor motor;

    private double gearing;

    private double momentOfInertia;

    private HardLimits limits;

    private double mass;

    private double centerOfMass;

    private SimMotorController controller;

    /**
    * Creates a new ArmMechanism instance.
    *
    * @param motor The motor used to actuate the arm.
    * @param controller The motor controller for the arm.
    * @param gearing The gear ratio of the arm (output speed / input speed).
    * @param momentOfInertia The moment of inertia of the arm in kg m^2.
    * @param minAngle The minimum angle of the arm in radians.
    * @param maxAngle The maximum angle of the arm in radians.
    * @param weight The weight of the arm in kg.
    * @param centerOfMass The center of mass of the arm in meters from the pivot.
    */

    public ArmMechanism(

    DCMotor motor,

    SimMotorController controller,

    double gearing,

    double momentOfInertia,

    HardLimits limits,

    double weight,

    double centerOfMass

    ) {

    this.motor = motor;

    this.gearing = gearing;

    this.momentOfInertia = momentOfInertia;

    this.limits = limits;

    this.mass = weight;

    this.centerOfMass = centerOfMass;

    this.controller = controller;

    }



    @Override

    public MechanismState getState() {

    DogLog.log("Sim/Arm/Position", this.state.getPosition());

    DogLog.log("Sim/Arm/Velocity", this.state.getVelocity());

    DogLog.log("Sim/Arm/Acceleration", this.state.getAcceleration());

    return this.state;

    }

    @Override

    public Torques getTorques() {
        torques.mTorque = motorTorque;
        torques.gTorque = gravityTorque;
        torques.bTorque = brakingTorque;
        return torques;
    }



    @Override

    public void update(double dt) {

    // For now, assume a fixed 12V supply

    var output = this.controller.run(dt, 12.0);
    DogLog.log("Sim/Arm/Voltage", output.voltage());
    DogLog.log("Sim/Arm/Current", output.current());
    double mechanismVelocity = state.getVelocity();//rad/s
    double motorVelocity = (mechanismVelocity * this.gearing);//rad/s
    // 2. Calculate motor torque based on the controller output type
    if (output.useCurrent()) {

        // Controller is providing a target current

        motorTorque = this.motor.getTorque(output.current())*this.gearing;

    } else {

        // Controller is providing a target voltage
        if(controller.isBrakeMode() && this.state.getVelocity() != 0.0){
            brakingTorque = -(this.state.getVelocity()/Math.abs(this.state.getVelocity())) * 0.01144 * Math.abs(this.state.getVelocity()) * this.gearing;
        }
            

        double appliedVoltage = output.voltage();

        if (appliedVoltage == 0.0){
            motorTorque = 0.0;
        }
        else{
            motorTorque = this.motor.getTorque(this.motor.getCurrent(motorVelocity, appliedVoltage))*this.gearing;
        }

        DogLog.log("Sim/Arm/Motor Torque", motorTorque);
        DogLog.log("Sim/Arm/Braking Force", brakingTorque);

    }


    // 3. Calculate physics

    final double g = -9.8;

    gravityTorque = this.centerOfMass * this.mass * g * Math.cos(state.getPosition()); 

    // double totalTorque = gravityTorque +(motorTorque * this.gearing) + brakingTorque;
    double totalTorque = gravityTorque+brakingTorque+motorTorque;





    // 4. Update state via Euler integration

    this.state.setAcceleration(totalTorque / this.momentOfInertia);

    this.state.setVelocity(this.state.getVelocity() + this.state.getAcceleration() * dt);

    this.state.setPosition(this.state.getPosition() + this.state.getVelocity() * dt);



    // 5. Apply constraints

    if (this.state.getPosition() > limits.getMax() && !limits.isUnbounded()) {

        this.state.setPosition(limits.getMax());

        this.state.setVelocity(0.0);
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", false);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", true);

    } else if (this.state.getPosition() < limits.getMin() && !limits.isUnbounded()) {

        this.state.setPosition(limits.getMin());

        this.state.setVelocity(0.0);
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", true);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", false);

    } else{
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", false);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", false);
    }

    }

}