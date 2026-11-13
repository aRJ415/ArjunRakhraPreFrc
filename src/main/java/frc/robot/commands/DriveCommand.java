package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;

/** Drives the robot with values from the controller. Used as the Drive subsystem's default. */
public class DriveCommand extends Command {
  private final Drive preDrive;
  private final DoubleSupplier prespeed;
  private final DoubleSupplier prerotation;

  /**
   * @param drive the Drive subsystem
   * @param speed supplies forward/back speed, -1.0 to 1.0
   * @param rotation supplies turn speed, -1.0 to 1.0 (positive = left)
   */
  public DriveCommand(Drive drive, DoubleSupplier speed, DoubleSupplier rotation) {
    preDrive = drive;
    prespeed = speed;
    prerotation = rotation;

    // REQUIRED: tells the scheduler this command uses the drivetrain.
    // setDefaultCommand() will crash the robot code without it.
    addRequirements(preDrive);
  }

  /** Called every 20 ms - reads the controller and drives. */
  @Override
  public void execute() {
    preDrive.arcadeDrive(prespeed.getAsDouble(), prerotation.getAsDouble());
  }

  /** Called when the command ends or gets interrupted. */
  @Override
  public void end(boolean interrupted) {
    preDrive.stop();
  }
}