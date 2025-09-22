package utils.sim.mechanisms;



import dev.doglog.DogLog;

import edu.wpi.first.math.system.plant.DCMotor;

import utils.sim.MechanismState;

import utils.sim.SimFrameWork;

import utils.sim.hardwareSim.SimMotorController;



public class ArmMechanism extends SimFrameWork{

    private MechanismState state = new MechanismState(Math.PI/2, 0.0, 0.0);

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

    public MechanismState getState() {

    DogLog.log("Sim/Arm/Position", this.state.getPosition());

    DogLog.log("Sim/Arm/Velocity", this.state.getVelocity());

    DogLog.log("Sim/Arm/Acceleration", this.state.getAcceleration());

    return this.state;

    }



    @Override

    public void update(double dt) {

    // For now, assume a fixed 12V supply

    var output = this.controller.run(dt, 12.0);
    double motorTorque;
    double mechanismVelocity = state.getVelocity();//rad/s
    double motorVelocity = (mechanismVelocity * this.gearing)/(Math.PI/2);//rot/s
    // 2. Calculate motor torque based on the controller output type
    if (output.useCurrent()) {

        // Controller is providing a target current

        motorTorque = this.motor.getTorque(output.current());

    } else {

        // Controller is providing a target voltage

        double appliedVoltage = output.voltage();



        // If brake mode is on and commanded voltage is near zero, apply strong resistance

        if (this.controller.isBrakeMode() && Math.abs(appliedVoltage) < 1e-2) {

        this.state.setVelocity(this.state.getVelocity() * 0.1);

        appliedVoltage = 0;

        }


        motorTorque = this.motor.getTorque(this.motor.getCurrent(motorVelocity, appliedVoltage));

    }


    // 3. Calculate physics

    final double g = 9.8;

    double gravityTorque = this.centerOfMass * this.weight * g * Math.cos(state.getPosition());

    double totalTorque = -gravityTorque;



    // 4. Update state via Euler integration

    this.state.setAcceleration(totalTorque / this.momentOfInertia);

    this.state.setVelocity(this.state.getVelocity() + this.state.getAcceleration() * dt);

    this.state.setPosition(this.state.getPosition() + this.state.getVelocity() * dt);



    // 5. Apply constraints

    if (this.state.getPosition() > this.maxAngle) {

        this.state.setPosition(this.maxAngle);

        this.state.setVelocity(0.0);
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", false);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", true);

    } else if (this.state.getPosition() < this.minAngle) {

        this.state.setPosition(this.minAngle);

        this.state.setVelocity(0.0);
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", true);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", false);

    } else{
        DogLog.log("Sim/Arm/Is At Hard Reverse Limit", false);
        DogLog.log("Sim/Arm/Is At Hard Forward Limit", false);
    }

    }

}