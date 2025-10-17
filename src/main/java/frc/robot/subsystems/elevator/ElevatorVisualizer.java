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
    private final Mechanism2d elevatorVisualizer = new Mechanism2d(20, 20);
    private final MechanismRoot2d elevatorRoot = elevatorVisualizer.getRoot("Elevator", 10, 0);
    private final MechanismLigament2d elevatorCarriage = elevatorRoot.append(new MechanismLigament2d("Carriage", 10, 90, 5, new Color8Bit(255, 0, 0)));
    private final MechanismLigament2d elevatorGoal = elevatorRoot.append(new MechanismLigament2d("Goal", 0, 90, 10, new Color8Bit(0, 255, 0)));

    MechanismRoot2d gRoot2d = elevatorVisualizer.getRoot("gTorque", 1, 10);
    MechanismLigament2d gTorqueArm = gRoot2d.append(new MechanismLigament2d("gTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d mRoot2d = elevatorVisualizer.getRoot("mTorque", 3, 10);
    MechanismLigament2d mTorqueArm = mRoot2d.append(new MechanismLigament2d("mTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d bRoot2d = elevatorVisualizer.getRoot("bTorque", 5, 10);
    MechanismLigament2d bTorqueArm = bRoot2d.append(new MechanismLigament2d("bTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d tRoot2d = elevatorVisualizer.getRoot("tTorque", 7, 10);
    MechanismLigament2d tTorqueArm = tRoot2d.append(new MechanismLigament2d("tTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up

    public double interpolateHeightFromTorques(double torque){
        // Interpolate the height of the torque arm based on the torque value
        // Assuming max torque is 10 Nm for scaling purposes
        double maxTorque = 170;
        double minHeight = 0.0;
        double maxHeight = 5.0;
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

    public ElevatorVisualizer(){
        elevatorCarriage.setColor(new Color8Bit(255, 0, 0));
        elevatorGoal.setColor(new Color8Bit(0, 255, 0));
        elevatorGoal.setLineWeight(30);
        elevatorCarriage.setLineWeight(15);
        
        elevatorVisualizer.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("ELEVATOR VISUALIZER", elevatorVisualizer);
    }
    public ElevatorVisualizer(boolean simulation){
        elevatorCarriage.setColor(new Color8Bit(255, 0, 0));
        elevatorGoal.setColor(new Color8Bit(0, 255, 0));
        mTorqueArm.setColor(new Color8Bit(Color.kBlue));
        gTorqueArm.setColor(new Color8Bit(Color.kGreen));
        bTorqueArm.setColor(new Color8Bit(Color.kYellow));
        tTorqueArm.setColor(new Color8Bit(Color.kWhite));
        elevatorVisualizer.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("ELEVATOR VISUALIZER", elevatorVisualizer);
    }

    public void update(double heightMeters, double goalMeters, Torques torques, boolean positionControlled){
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Updated to height of: ", heightMeters);
        DogLog.log("Robot/Subsystems/Elevator/Visualizer/Goal height of: ", goalMeters);
        elevatorCarriage.setLength(heightMeters + 6); // +6 to offset the base of the elevator
        gTorqueArm.setLength(interpolateHeightFromTorques(torques.gTorque));
        mTorqueArm.setLength(interpolateHeightFromTorques(torques.mTorque));
        bTorqueArm.setLength(interpolateHeightFromTorques(torques.bTorque));
        tTorqueArm.setLength(interpolateHeightFromTorques(torques.gTorque + torques.mTorque + torques.bTorque));

        gTorqueArm.setAngle(interpolateDirectionFromTorques(torques.gTorque));
        mTorqueArm.setAngle(interpolateDirectionFromTorques(torques.mTorque));
        bTorqueArm.setAngle(interpolateDirectionFromTorques(torques.bTorque));
        tTorqueArm.setAngle(interpolateDirectionFromTorques(torques.gTorque + torques.mTorque + torques.bTorque));
        
        if (positionControlled){
            this.elevatorGoal.setColor(new Color8Bit(Color.kGreen));
            this.elevatorGoal.setLength(goalMeters+6);
        } else {
            this.elevatorGoal.setColor(new Color8Bit(Color.kBlack));
            this.elevatorGoal.setLength(0);
        }
    }

}
