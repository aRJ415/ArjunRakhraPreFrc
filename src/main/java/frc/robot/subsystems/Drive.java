package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * The drivetrain. Each side has 3 wheels chained together, driven through the middle wheel by a
 * 2-motor gearbox: one LEADER motor (we command it) and one FOLLOWER (it copies the leader).
 */
public class Drive extends SubsystemBase {
  // ---- Motors ----
  private final SparkFlex leftLeader =
      new SparkFlex(DriveConstants.kLeftLeaderID, MotorType.kBrushless);
  private final SparkFlex leftFollower =
      new SparkFlex(DriveConstants.kLeftFollowerID, MotorType.kBrushless);
  private final SparkFlex rightLeader =
      new SparkFlex(DriveConstants.kRightLeaderID, MotorType.kBrushless);
  private final SparkFlex rightFollower =
      new SparkFlex(DriveConstants.kRightFollowerID, MotorType.kBrushless);

  // ---- Encoders (built into the Vortex, read from the leaders) ----
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive predrive;

  public Drive() {
    // Configure the motors FIRST, before anything tries to drive them.
    configureMotors();

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // Only the leaders go into DifferentialDrive. The followers copy their leaders on their own.
    predrive = new DifferentialDrive(leftLeader, rightLeader);
  }

  private void configureMotors() {
    // Settings shared by all 4 motors.
    SparkFlexConfig baseConfig = new SparkFlexConfig();
    baseConfig
        .idleMode(IdleMode.kBrake) // brake when you let go of the controls
        .smartCurrentLimit(DriveConstants.kCurrentLimit)
        .voltageCompensation(12.0); // drive the same on a full or tired battery

    // Leaders: shared settings + which direction is "forward" for that side.
    SparkFlexConfig leftLeaderConfig = new SparkFlexConfig();
    leftLeaderConfig.apply(baseConfig);
    leftLeaderConfig.inverted(DriveConstants.kLeftInverted);

    SparkFlexConfig rightLeaderConfig = new SparkFlexConfig();
    rightLeaderConfig.apply(baseConfig);
    rightLeaderConfig.inverted(DriveConstants.kRightInverted);

    // Followers: shared settings + "copy this leader".
    // Both motors in a gearbox face the same way, so the follower is NOT inverted relative to
    // its leader. (If your gearbox has them facing opposite ways, use follow(leader, true).)
    SparkFlexConfig leftFollowerConfig = new SparkFlexConfig();
    leftFollowerConfig.apply(baseConfig);
    leftFollowerConfig.follow(leftLeader);

    SparkFlexConfig rightFollowerConfig = new SparkFlexConfig();
    rightFollowerConfig.apply(baseConfig);
    rightFollowerConfig.follow(rightLeader);

    // Send each config to its motor controller.
    //   kResetSafeParameters = start from factory defaults (a swapped controller acts the same)
    //   kPersistParameters   = save to flash (settings survive a brownout)
    leftLeader.configure(
        leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Drives the robot.
   *
   * @param speed forward/back, -1.0 to 1.0 (positive = forward)
   * @param turn rotation, -1.0 to 1.0 (positive = turn left / counterclockwise)
   */
  public void arcadeDrive(double speed, double turn) {
    predrive.arcadeDrive(speed, turn);
  }

  /** Stops the drive motors. */
  public void stop() {
    predrive.stopMotor();
  }

  @Override
  public void periodic() {
    // Runs every 20 ms. Watch these on the dashboard to check your motor directions:
    // when the robot drives forward, BOTH numbers should go up.
    SmartDashboard.putNumber("Drive/Left Position (rot)", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive/Right Position (rot)", rightEncoder.getPosition());
  }
}