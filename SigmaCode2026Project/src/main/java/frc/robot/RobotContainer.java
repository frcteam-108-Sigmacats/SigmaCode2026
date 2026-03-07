package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.DefaultSpinDexerCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.TransferFuelToShooter;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.SpinDexer.SpinDexerIOReal;
import frc.robot.subsystems.SpinDexer.SpinDexerIOSim;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOMix;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooterMech;
  private final Drive swerveDrive;
  private final IntakeMech intakeMech;

  private CommandXboxController driver = new CommandXboxController(0);
  private Trigger bLT, bRT;
  private final SpinDexerMech spinDexerMech;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("AutoChooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        shooterMech = new Shooter(new ShooterIOReal());
        spinDexerMech = new SpinDexerMech(new SpinDexerIOReal());
        intakeMech = new IntakeMech(new IntakeIOReal());
        swerveDrive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMix(0),
                new ModuleIOMix(1),
                new ModuleIOMix(2),
                new ModuleIOMix(3));

        break;
      default:
        shooterMech = new Shooter(new ShooterIOReal());
        spinDexerMech = new SpinDexerMech(new SpinDexerIOSim());
        intakeMech = new IntakeMech(new IntakeIOSim());
        swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;
    }
    // Configure the trigger bindings
    configureBindings();

    shooterMech.setDefaultCommand(new DefaultShooter(shooterMech, swerveDrive));
    spinDexerMech.setDefaultCommand(new DefaultSpinDexerCommand(spinDexerMech));
    intakeMech.setDefaultCommand(new DefaultIntakeCommand(intakeMech));
    swerveDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerveDrive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Configure the trigger bindings
    configureBindings();
    bLT.whileTrue(new TransferFuelToShooter(spinDexerMech, swerveDrive));
    bRT.whileTrue(new RunIntakeCommand(intakeMech, swerveDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    bRT = driver.rightTrigger();
    bLT = driver.leftTrigger();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public void updateSimulation() {}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Exposes the turret subsystem so {@link Robot} can seed its hood encoder on teleop init. */
  // public Shooter getTurret() {
  //   return getTurret();
  // }
}
