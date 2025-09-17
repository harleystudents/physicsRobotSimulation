package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.wrist.Wrist;

public class DriverController {
    private final CommandXboxController controller;

  /** Button: 1 */
  protected final Trigger A;

  /** Button: 2 */
  protected final Trigger B;

  /** Button: 3 */
  protected final Trigger X;

  /** Button: 4 */
  protected final Trigger Y;

  /** Left Center; Button: 7 */
  protected final Trigger Back;

  /** Right Center; Button: 8 */
  protected final Trigger Start;

  /** Left Bumper; Button: 5 */
  protected final Trigger LB;

  /** Right Bumper; Button: 6 */
  protected final Trigger RB;

  /** Left Stick; Button: 9 */
  protected final Trigger LS;

  /** Right Stick; Button: 10 */
  protected final Trigger RS;

  /** Left Trigger; Axis: 2 */
  protected final Trigger LT;

  /** Right Trigger; Axis: 3 */
  protected final Trigger RT;

  /** DPad Up; Degrees: 0 */
  protected final Trigger DPU;

  /** DPad Right; Degrees: 90 */
  protected final Trigger DPR;

  /** DPad Down; Degrees: 180 */
  protected final Trigger DPD;

  /** DPad Left; Degrees: 270 */
  protected final Trigger DPL;

  public DriverController(int port) {
    controller = new CommandXboxController(port);

    A = controller.a();
    B = controller.b();
    X = controller.x();
    Y = controller.y();
    Back = controller.back();
    Start = controller.start();
    LB = controller.leftBumper();
    RB = controller.rightBumper();
    LS = controller.leftStick();
    RS = controller.rightStick();
    LT = controller.leftTrigger(0.1);
    RT = controller.rightTrigger(0.1);
    DPU = controller.povUp();
    DPR = controller.povRight();
    DPD = controller.povDown();
    DPL = controller.povLeft();
  }

  public void bind(Wrist wrist){
    this.A.onTrue(WristCommands.occilate(wrist));
  }

}
