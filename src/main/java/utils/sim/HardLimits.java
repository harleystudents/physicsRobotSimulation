package utils.sim;

public class HardLimits {
    private final boolean unbounded;
    double min;
    double max;
    public HardLimits(boolean unbounded){
        this.unbounded = unbounded;
    }
    public HardLimits(double min, double max){
        this.unbounded = false;
        this.min = min;
        this.max = max;
    }
    public boolean isUnbounded(){
        return unbounded;
    }
    public double getMin(){
        return min;
    }
    public double getMax(){
        return max;
    }
}
