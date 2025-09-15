package utils.sim;

public abstract class SimFrameWork {
    public MechanismState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double position() { return position; }
    public double velocity() { return velocity; }
    public double acceleration() { return acceleration; }
}
    abstract public void update(double dt);
    abstract public void getState()
    
    
}
