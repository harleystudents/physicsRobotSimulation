package frc.robot.subsystems.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import utils.sim.Torques;

public class WristVisualizer {
    Mechanism2d wristMechanism = new Mechanism2d(30, 30);
    MechanismRoot2d wristBase = wristMechanism.getRoot("Wrist Base", 15, 15);
    MechanismLigament2d arm = wristBase.append(new MechanismLigament2d("Arm", 10, 0));
    MechanismLigament2d goal = wristBase.append(new MechanismLigament2d("Goal", 10, 0, 5, new Color8Bit(Color.kBlack)));
    MechanismRoot2d gRoot2d = wristMechanism.getRoot("gTorque", 1, 10);
    MechanismLigament2d gTorqueArm = gRoot2d.append(new MechanismLigament2d("gTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d mRoot2d = wristMechanism.getRoot("mTorque", 3, 10);
    MechanismLigament2d mTorqueArm = mRoot2d.append(new MechanismLigament2d("mTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d bRoot2d = wristMechanism.getRoot("bTorque", 5, 10);
    MechanismLigament2d bTorqueArm = bRoot2d.append(new MechanismLigament2d("bTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up
    MechanismRoot2d tRoot2d = wristMechanism.getRoot("tTorque", 7, 10);
    MechanismLigament2d tTorqueArm = tRoot2d.append(new MechanismLigament2d("tTorque", 5, 90, 5, new Color8Bit(Color.kBlack))); // black so does not show up

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


    public WristVisualizer(){
        arm.setColor(new Color8Bit(255, 0, 0));
        wristMechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("WRIST MECHANISM", wristMechanism);
    }
    public WristVisualizer(boolean simulation){
        arm.setColor(new Color8Bit(255, 0, 0));
        mTorqueArm.setColor(new Color8Bit(Color.kBlue));
        gTorqueArm.setColor(new Color8Bit(Color.kGreen));
        bTorqueArm.setColor(new Color8Bit(Color.kYellow));
        tTorqueArm.setColor(new Color8Bit(Color.kWhite));
        wristMechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData("WRIST MECHANISM", wristMechanism);

    }
    public void update(double angle){
        //The angle is in degrees
        DogLog.log("Robot/Subsystems/Wrist/Visualizer/Updated to angle of: ", angle);
        arm.setAngle(angle);
    }
    public void update(double angle, double goal, boolean positionControlled, Torques torques){
        //The angle is in degrees
        DogLog.log("Robot/Subsystems/Wrist/Visualizer/Updated to angle of: ", angle);
        arm.setAngle(angle);
        gTorqueArm.setLength(interpolateHeightFromTorques(torques.gTorque));
        mTorqueArm.setLength(interpolateHeightFromTorques(torques.mTorque));
        bTorqueArm.setLength(interpolateHeightFromTorques(torques.bTorque));
        tTorqueArm.setLength(interpolateHeightFromTorques(torques.gTorque + torques.mTorque + torques.bTorque));

        gTorqueArm.setAngle(interpolateDirectionFromTorques(torques.gTorque));
        mTorqueArm.setAngle(interpolateDirectionFromTorques(torques.mTorque));
        bTorqueArm.setAngle(interpolateDirectionFromTorques(torques.bTorque));
        tTorqueArm.setAngle(interpolateDirectionFromTorques(torques.gTorque + torques.mTorque + torques.bTorque));
        
        if (positionControlled){
            this.goal.setColor(new Color8Bit(Color.kGreen));
            this.goal.setLength(10);
            this.goal.setAngle(goal);
        } else {
            this.goal.setColor(new Color8Bit(Color.kBlack));
            this.goal.setLength(0);
        }

        // DogLog.log("Robot/Subsystems/Wrist/Visualizer/Gravity Torque", gTorque);
        // DogLog.log("Robot/Subsystems/Wrist/Visualizer/Motor Torque", mTorque);
        // DogLog.log("Robot/Subsystems/Wrist/Visualizer/Brake Torque", bTorque);
        // DogLog.log("Robot/Subsystems/Wrist/Visualizer/Total Torque", gTorque + mTorque + bTorque);
    }

}
