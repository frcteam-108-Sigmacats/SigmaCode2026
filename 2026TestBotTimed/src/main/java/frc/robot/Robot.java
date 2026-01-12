// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.KxNSwerve.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  //Set up for some auto routines to add into a sendable chooser so user can pick which auto to run
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Creating the xbox controller to drive the robot
  private final CommandXboxController driver = new CommandXboxController(0); //0 is the first controller that is plugged into the computer
  //Grabbing the swerve drive that will run all the wheels
  private final SwerveDrive swerveDrive = new SwerveDrive();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Grabs the Left joystick x and y direction that the user is pushing the joystick to (Has the robot drive to that direction that the joystick is facing)
    double xAxis = -driver.getLeftX();
    double yAxis = -driver.getLeftY();
    //Grabs the x direction of the right joystick that the user is pushing the joystick to (Rotates the whole robot)
    double rotAxis = -driver.getRightX();
    //Checks if the driver is pushing the joystick more than 20% of its full throttle just to check for stick drift
    xAxis = (Math.abs(xAxis) < 0.2) ? 0.0 : xAxis * 4.0; // Tertiary statement (inline if statement) Ex: Condition ? if true run this task : if not true run this task
    yAxis = (Math.abs(yAxis) < 0.2) ? 0.0 : yAxis * 4.0;
    rotAxis = (Math.abs(rotAxis) < 0.2) ? 0.0 : rotAxis * 2 * Math.PI;
    //Stores the x and y axis in a translation variable
    Translation2d translation = new Translation2d(yAxis, xAxis);
    //Calling the drive task inside SwerveDrive class to have the robot use the data from the joysticks to tell the robot where its supposed to be going
    swerveDrive.drive(translation, rotAxis, true);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
