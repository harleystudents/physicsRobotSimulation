package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import utils.sim.Torques;

public class ElevatorVisualizer {

    public double interpolateHeightFromTorques(double torque){
        // Interpolate the height of the torque arm based on the torque value
        // Assuming max torque is 10 Nm for scaling purposes
        double maxTorque = 10.0;
        double minHeight = 0.0;
        double maxHeight = 10.0;
        if (torque > maxTorque) torque = maxTorque;
        if (torque < -maxTorque) torque = -maxTorque;
        return minHeight + (Math.abs(torque) / maxTorque) * (maxHeight - minHeight);
    }

    public double interpolateDirectionFromTorques(double torque){
        // Interpolate the direction of the torque arm based on the torque value
        // Positive torque is clockwise (90 degrees), negative torque is counter-clockwise (270 degrees)
        if (torque >= 0) {
            return 90.0;
        } else {
            return 270.0;
        }
    }

    private Mechanism2d elevatorMechanism2d = new Mechanism2d(30, 30);
    private MechanismRoot2d elevatorBase = elevatorMechanism2d.getRoot("Elevator Base", 8, 0);
    private MechanismLigament2d elevator = elevatorBase.append(new MechanismLigament2d("Elevator", 15, 90, 5, new Color8Bit(Color.kRed)));
    private MechanismLigament2d goal = elevatorBase.append(new MechanismLigament2d("Goal", 15, 90, 5, new Color8Bit(Color.kGreen)));
    private MechanismRoot2d gRoot2d = elevatorMechanism2d.getRoot("gTorque", 1, 10);
    private MechanismLigament2d gTorqueArm = gRoot2d.append(new MechanismLigament2d("gTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    private MechanismRoot2d mRoot2d = elevatorMechanism2d.getRoot("mTorque", 3, 10);
    private MechanismLigament2d mTorqueArm = mRoot2d.append(new MechanismLigament2d("mTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    private MechanismRoot2d bRoot2d = elevatorMechanism2d.getRoot("bTorque", 5, 10);
    private MechanismLigament2d bTorqueArm = bRoot2d.append(new MechanismLigament2d("bTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    private MechanismRoot2d tRoot2d = elevatorMechanism2d.getRoot("tTorque", 7, 10);
    private MechanismLigament2d tTorqueArm = tRoot2d.append(new MechanismLigament2d("tTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
   
    public ElevatorVisualizer() {
        elevator.setColor(new Color8Bit(Color.kRed));
        elevatorMechanism2d.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("ELEVATOR MECHANISM", elevatorMechanism2d);
    }
  
    public ElevatorVisualizer(boolean isSimulation){
        elevator.setColor(new Color8Bit(Color.kRed));
        mTorqueArm.setColor(new Color8Bit(Color.kBlue));
        gTorqueArm.setColor(new Color8Bit(Color.kGreen));
        bTorqueArm.setColor(new Color8Bit(Color.kYellow));
        tTorqueArm.setColor(new Color8Bit(Color.kWhite));
        elevatorMechanism2d.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("ELEVATOR MECHANISM", elevatorMechanism2d);
    }
    public void update(double heightMeters, double goalHeightMeters){
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Updated to height of: ", heightMeters);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Goal height of: ", goalHeightMeters);
        elevator.setLength(heightMeters);
        goal.setLength(goalHeightMeters);

    }
    public void update(double heightMeters, double goalHeightMeters, Torques torques){
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Updated to height of: ", heightMeters);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Goal height of: ", goalHeightMeters);
        elevator.setLength(heightMeters+.2);
        goal.setLength(goalHeightMeters+.2);
        double gTorque = torques.gTorque;
        double mTorque = torques.mTorque;
        double bTorque = torques.bTorque;
        double tTorque = torques.bTorque + torques.mTorque + torques.gTorque;
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Gravity Torque: ", gTorque);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Motor Torque: ", mTorque);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Bearing Torque: ", bTorque);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Total Torque: ", tTorque);
        gTorqueArm.setLength(interpolateHeightFromTorques(gTorque));
        gTorqueArm.setAngle(interpolateDirectionFromTorques(gTorque));
        mTorqueArm.setLength(interpolateHeightFromTorques(mTorque));
        mTorqueArm.setAngle(interpolateDirectionFromTorques(mTorque));
        bTorqueArm.setLength(interpolateHeightFromTorques(bTorque));
        bTorqueArm.setAngle(interpolateDirectionFromTorques(bTorque));
        tTorqueArm.setLength(interpolateHeightFromTorques(tTorque));
        tTorqueArm.setAngle(interpolateDirectionFromTorques(tTorque));
    }
}
