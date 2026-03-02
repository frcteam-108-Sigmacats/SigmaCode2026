// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import java.util.Set;
// import java.util.function.DoubleSupplier;

// /**
//  * Tracks the alliance's Reef hub using the Limelight and rotates the robot to face it while
// still
//  * allowing the driver to translate freely.
//  *
//  * <p>2026 Reefscape AprilTag layout used for hub identification:
//  *
//  * <ul>
//  *   <li>Red hub face tags : 6, 7, 8, 9, 10, 11
//  *   <li>Blue hub face tags : 17, 18, 19, 20, 21, 22
//  * </ul>
//  *
//  * <p>The Limelight is told to prioritise the nearest alliance tag so the robot always locks on
// to
//  * the closest reef face regardless of which direction it started from. When no valid target is
//  * visible the driver retains full control.
//  */
// public class TrackHubCommand extends Command {

//   // ── hub AprilTag sets ───────────────────────────────────────────────────
//   private static final Set<Integer> RED_HUB_TAGS = Set.of(6, 7, 8, 9, 10, 11);
//   private static final Set<Integer> BLUE_HUB_TAGS = Set.of(17, 18, 19, 20, 21, 22);

//   // ── Speeds ───────────────────────────────────────────────────────────────
//   private static final double MAX_SPEED_MPS = 4.5;
//   private static final double MAX_ANGULAR_SPEED = Math.PI;
//   private static final double DEADBAND = 0.1;

//   // ── Rotation PID ─────────────────────────────────────────────────────────
//   // kP chosen so that a 10° offset → ~0.5 rad/s correction; tune on robot.
//   private static final double ROT_kP = 0.05;
//   private static final double ROT_kI = 0.0;
//   private static final double ROT_kD = 0.001;
//   // Tolerance: consider "on target" when TX error < 1°
//   private static final double ROT_TOLERANCE_DEG = 1.0;

//   private final SwerveSubsystem swerve;
//   private final LimelightSubsystem limelight;
//   private final DoubleSupplier xSupplier; // driver forward/back translation
//   private final DoubleSupplier ySupplier; // driver left/right strafe
//   private final PIDController rotPID;

//   /**
//    * @param swerve the swerve subsystem
//    * @param limelight the Limelight subsystem (already filtering for alliance tags)
//    * @param xSupplier forward (+) / back (−) translation [−1, 1]
//    * @param ySupplier left (+) / right (−) strafe [−1, 1]
//    */
//   public TrackHubCommand(
//       SwerveSubsystem swerve,
//       LimelightSubsystem limelight,
//       DoubleSupplier xSupplier,
//       DoubleSupplier ySupplier) {

//     this.swerve = swerve;
//     this.limelight = limelight;
//     this.xSupplier = xSupplier;
//     this.ySupplier = ySupplier;

//     rotPID = new PIDController(ROT_kP, ROT_kI, ROT_kD);
//     rotPID.setTolerance(ROT_TOLERANCE_DEG);
//     // Setpoint is 0° – we want TX (horizontal offset) to be zero.
//     rotPID.setSetpoint(0.0);

//     addRequirements(swerve);
//   }

//   // ── Alliance helpers ─────────────────────────────────────────────────────

//   /**
//    * Returns the set of Reef AprilTag IDs that belong to the robot's current alliance. Falls back
// to
//    * Red tags if alliance is not yet determined.
//    */
//   private Set<Integer> getAllianceHubTags() {
//     var alliance = DriverStation.getAlliance();
//     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
//       return BLUE_HUB_TAGS;
//     }
//     return RED_HUB_TAGS; // default / Red
//   }

//   /**
//    * Returns the direction the robot should shoot toward (i.e. the opposing alliance's hub
//    * direction). Used for dashboard telemetry or a future shooter subsystem.
//    */
//   public Set<Integer> getShootTargetTags() {
//     var alliance = DriverStation.getAlliance();
//     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
//       return RED_HUB_TAGS; // Blue alliance shoots toward Red hub
//     }
//     return BLUE_HUB_TAGS; // Red alliance shoots toward Blue hub
//   }

//   // ── Command lifecycle ────────────────────────────────────────────────────

//   @Override
//   public void initialize() {
//     rotPID.reset();
//     // Tell the Limelight which tags to prioritise for this command
//     limelight.setTargetTags(getAllianceHubTags());
//   }

//   @Override
//   public void execute() {
//     double xSpeed = applyDeadband(xSupplier.getAsDouble()) * MAX_SPEED_MPS;
//     double ySpeed = applyDeadband(ySupplier.getAsDouble()) * MAX_SPEED_MPS;

//     double rotSpeed;
//     if (limelight.hasTarget() && isAllianceTag(limelight.getTargetId())) {
//       // TX is positive when target is to the right; we rotate CCW (positive)
//       // to bring it back to centre, so we negate.
//       double tx = limelight.getTX();
//       rotSpeed = MathUtil.clamp(-rotPID.calculate(tx), -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
//     } else {
//       // No valid alliance tag visible – hold heading, let driver translate.
//       rotSpeed = 0.0;
//     }

//     // Always field-relative while tracking so the driver's stick directions are intuitive.
//     swerve.drive(xSpeed, ySpeed, rotSpeed, true);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     // Restore default tag filtering so normal vision-based odometry resumes.
//     limelight.clearTargetTagFilter();
//     swerve.stopModules();
//   }

//   @Override
//   public boolean isFinished() {
//     return false; // Runs until the button is released (whileTrue binding).
//   }

//   // ── Helpers ──────────────────────────────────────────────────────────────

//   private boolean isAllianceTag(int tagId) {
//     return getAllianceHubTags().contains(tagId);
//   }

//   private double applyDeadband(double value) {
//     return MathUtil.applyDeadband(value, DEADBAND);
//   }
// }
