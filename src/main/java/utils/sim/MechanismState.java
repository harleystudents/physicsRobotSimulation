package utils.sim;

public class MechanismState {
    private double position;
    private double velocity;
    private double acceleration;
    public MechanismState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }


    public double getPosition() { return position; }
    public double getVelocity() { return velocity; }
    public double getAcceleration() { return acceleration; }

    public double setAcceleration(double acceleration) { return this.acceleration = acceleration; }
    public double setVelocity(double velocity) { return this.velocity = velocity; }
    public double setPosition(double position) { return this.position = position; }
}
