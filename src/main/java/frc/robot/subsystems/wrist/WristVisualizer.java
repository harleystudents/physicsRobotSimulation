package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WristVisualizer {
    Mechanism2d wristMechanism = new Mechanism2d(30, 30);
    MechanismRoot2d wristBase = wristMechanism.getRoot("Wrist Base", 15, 15);
    MechanismLigament2d arm = wristBase.append(new MechanismLigament2d("Arm", 10, 0));

    public WristVisualizer(){
        arm.setColor(new Color8Bit(255, 0, 0));
        wristMechanism.setBackgroundColor(new Color8Bit(Color.kBlack));
        SmartDashboard.putData(wristMechanism);
    }
    public void update(double angle){
        arm.setAngle(angle);
    }

}
