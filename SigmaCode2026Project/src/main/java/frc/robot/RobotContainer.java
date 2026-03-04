package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.DefaultSpinDexerCommand;
import frc.robot.commands.ShootAndTransfer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.SpinDexer.SpinDexerIOReal;
import frc.robot.subsystems.SpinDexer.SpinDexerIOSim;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter shooterMech;

  private Trigger bA;
  private final SpinDexerMech spinDexerMech;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        shooterMech = new Shooter(new ShooterIOReal());
        spinDexerMech = new SpinDexerMech(new SpinDexerIOReal());

        break;
      default:
        shooterMech = new Shooter(new ShooterIOReal());
        spinDexerMech = new SpinDexerMech(new SpinDexerIOSim());

        break;
    }
    // Configure the trigger bindings
    configureBindings();

    shooterMech.setDefaultCommand(new DefaultShooter(shooterMech));

    // Configure the trigger bindings
    configureBindings();
    spinDexerMech.setDefaultCommand(new DefaultSpinDexerCommand(spinDexerMech));
    bA.whileTrue(new ShootAndTransfer(shooterMech, spinDexerMech));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    bA = m_driverController.a();

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public void updateSimulation() {
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
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
