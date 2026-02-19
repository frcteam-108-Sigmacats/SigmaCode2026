package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.spindexer.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final SpinDexer spinDexer;
  private final Shooter shooter;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate subsystems based on robot mode
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOReal());
       // intake = new Intake(new IntakeIOReal());
       // spinDexer = new SpinDexer(new SpinDexerIOReal());
        shooter = new Shooter(new ShooterIOReal());
        break;
      case SIM:
        // Simulation robot, instantiate physics sim implementations
        drive = new Drive(new DriveIOSim());
       // intake = new Intake(new IntakeIOSim());
        //spinDexer = new SpinDexer(new SpinDexerIOSim());
        shooter = new Shooter(new ShooterIOSim());
        break;
      default:
        // Replaying log, instantiate no-op implementations
        drive = new Drive(new DriveIO() {});
       // intake = new Intake(new IntakeIO() {});
       // spinDexer = new SpinDexer(new SpinDexerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Configure the button bindings
    configureBindings();

    // Configure default commands
    configureDefaultCommands();
  }

  /** Configures default commands for subsystems. */
  private void configureDefaultCommands() {
    // Drive with joysticks by default
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -applyDeadband(driverController.getLeftY(), OperatorConstants.kDriveDeadband),
            () -> -applyDeadband(driverController.getLeftX(), OperatorConstants.kDriveDeadband),
            () ->
                -applyDeadband(driverController.getRightX(), OperatorConstants.kRotationDeadband)));

    // Idle shooter by default
    shooter.setDefaultCommand(shooter.idleCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link CommandXboxController}'s methods.
   */
  private void configureBindings() {
    // Driver controls
    driverController.a().whileTrue(new DefenseModeCommand(drive));
    driverController.b().onTrue(ResetPoseCommand.toAllianceStart(drive));
    driverController.x().onTrue(ResetPoseCommand.resetPositionOnly(drive));

    // Operator controls - Intake
   // operatorController.rightTrigger().whileTrue(new AutoIntakeCommand(intake, spinDexer));

   // operatorController.leftTrigger().whileTrue(intake.outtakeCommand());

    // operatorController.back().whileTrue(new EjectAllCommand(intake, spinDexer));

    // Operator controls - Shooter (with auto-aim)
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                AutoAimCommand.aimAtSpeaker(
                        drive,
                       // shooter,
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue)
                    .withTimeout(2.0),
                //ShootSequenceCommand.forSpeaker(shooter, spinDexer)));

    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                AutoAimCommand.aimAtAmp(
                        drive,
                        //shooter,
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue)
                    .withTimeout(1.0),
               // ShootSequenceCommand.forAmp(shooter, spinDexer)));

    // Operator controls - Manual spin dexer
    operatorController.a().onTrue(spinDexer.indexCommand());
    operatorController.b().onTrue(Commands.runOnce(() -> spinDexer.rotateToSlot(0)));
    operatorController.x().onTrue(Commands.runOnce(() -> spinDexer.rotateToSlot(1)));
    operatorController.y().onTrue(Commands.runOnce(() -> spinDexer.rotateToSlot(2)));

    // Operator POV - Auto scoring
    operatorController.povUp().onTrue(AutoScoreCommand.scoreSpeaker(drive, shooter, spinDexer));
    operatorController.povDown().onTrue(AutoScoreCommand.scoreAmp(drive, shooter, spinDexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Example auto sequence using new commands
    return Commands.sequence(
        // Reset to alliance starting position
        ResetPoseCommand.toAllianceStart(drive),

        // Example PathPlanner auto (replace with your actual path)
        new PathPlannerAuto("ExampleAuto"),

        // Or build custom sequence:
        // Drive and intake first note
        // new FollowPathAndIntakeCommand(drive, intake, spinDexer, "ToFirstNote"),
        // Score in speaker
        // AutoScoreCommand.scoreSpeaker(drive, shooter, spinDexer),
        // Repeat for additional notes...

        // End in defense position
        new DefenseModeCommand(drive));
  }

  /** Applies deadband to controller input. */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
  }

  /** Gets the driver controller for use in commands. */
  public CommandXboxController getDriverController() {
    return driverController;
  }

  /** Gets the operator controller for use in commands. */
  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  /** Gets the drive subsystem for use in commands. */
  public Drive getDrive() {
    return drive;
  }

  /** Gets the intake subsystem for use in commands. */
  public Intake getIntake() {
    return intake;
  }

  /** Gets the spin dexer subsystem for use in commands. */
  public SpinDexer getSpinDexer() {
    return spinDexer;
  }

  /** Gets the shooter subsystem for use in commands. */
  public Shooter getShooter() {
    return shooter;
  }
}
