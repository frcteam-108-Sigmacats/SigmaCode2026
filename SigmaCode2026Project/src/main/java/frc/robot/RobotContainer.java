package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.DefaultSpinDexerCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveOverTheBump;
import frc.robot.commands.Outtaking;
import frc.robot.commands.ReverseSpinDexerCommand;
import frc.robot.commands.RunAll;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SlowMo;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterStatus;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
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
  private Shooter shooterMech;
  private Drive swerveDrive;
  private IntakeMech intakeMech;
  private SpinDexerMech spinDexerMech;
  private SlowMo slowMo;

  private CommandXboxController driver = new CommandXboxController(0);
  private Trigger bLT, bRT, bA, bB, bX, bY, dUP, dLEFTSTICK, dRIGHT, dDOWN, dSTART, bLB, bRB;
  private boolean slowMoActive = false;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

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
      case SIM:
        shooterMech = new Shooter(new ShooterIOSim());
        spinDexerMech = new SpinDexerMech(new SpinDexerIOSim());
        intakeMech = new IntakeMech(new IntakeIOSim());
        swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
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
    shooterMech.setDefaultCommand(new DefaultShooter(shooterMech, swerveDrive, false));
    // Uncomment this if the overrun loop stops showing up and go into the command and follow the
    // next instructions
    // shooterMech.setDefaultCommand(new DefaultShooter(shooterMech, swerveDrive::getPose,
    // swerveDrive::getDriveSpeedsFieldRelative, false));
    spinDexerMech.setDefaultCommand(new DefaultSpinDexerCommand(spinDexerMech));
    intakeMech.setDefaultCommand(new DefaultIntakeCommand(intakeMech));
    swerveDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerveDrive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> slowMoActive));

    // Configure the trigger bindings
    configureBindings();
    createAutoChooser();
    bLT.whileTrue(new RunAll(shooterMech, intakeMech, spinDexerMech, swerveDrive));
    bRT.whileTrue(new RunIntakeCommand(intakeMech, swerveDrive));
    bRB.whileTrue(new ReverseSpinDexerCommand(spinDexerMech));
    bLB.whileTrue(new Outtaking(intakeMech));
    // bY.whileTrue(new ReverseSpinDexerCommand(spinDexerMech));
    dLEFTSTICK.onTrue(
        new InstantCommand(
            () -> {
              if (!(swerveDrive.getDriveState() == ShooterStatus.SHOOT)
                  && !(swerveDrive.getDriveState() == ShooterStatus.PASSING)) {
                swerveDrive.setDriveState(ShooterStatus.INTAKE);
              }
            }));
    dLEFTSTICK.onFalse(
        new InstantCommand(
            () -> {
              if (swerveDrive.getDriveState() == ShooterStatus.INTAKE) {
                swerveDrive.setDriveState(ShooterStatus.DRIVE);
              }
            }));
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
    dSTART = driver.start();
    dDOWN = driver.povDown(); // climer down
    dUP = driver.povUp(); // climer up
    dLEFTSTICK = driver.leftStick();
    bRB = driver.rightBumper();
    bLB = driver.leftBumper();

    bA = driver.a();

    // Start/backpaddle button turns on slow-mo mode (30% speed reduction)
    bY = driver.y();
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

  public void createAutoChooser() {
    NamedCommands.registerCommand("Intake", new RunIntakeCommand(intakeMech, swerveDrive));
    NamedCommands.registerCommand(
        "RunAll", new RunAll(shooterMech, intakeMech, spinDexerMech, swerveDrive));
    NamedCommands.registerCommand(
        "RunOverBumpForward", new DriveOverTheBump(swerveDrive, "forward"));
    NamedCommands.registerCommand("RunOverBumpBack", new DriveOverTheBump(swerveDrive, "back"));
    NamedCommands.registerCommand("ResetPoseLLS", swerveDrive.resetPoseWithLLS());
    NamedCommands.registerCommand("StopSpinDexer", new DefaultSpinDexerCommand(spinDexerMech));
    // autoChooser.addDefaultOption("None", null);
    // autoChooser.addOption(
    //     "DepotAuto", new DepotAuto(swerveDrive, intakeMech, spinDexerMech, shooterMech));
    // autoChooser.addOption(
    //     "StationAuto", new StationAuto(swerveDrive, shooterMech, intakeMech, spinDexerMech));
    // autoChooser.addOption(
    //     "Test", new AutoTest(swerveDrive, intakeMech, spinDexerMech, shooterMech));
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
  }

  /** Exposes the turret subsystem so {@link Robot} can seed its hood encoder on teleop init. */
  // public Shooter getTurret() {
  //   return getTurret();
  // }
}
