// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  PIDController pidController = new PIDController(0.4, 0, 0.01);

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  static SwerveInputStream driveAngularVelocity;

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle;

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  double limelight_aim_proportional() {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = -.005;

    // Pass the name of your Limelight (e.g., "limelight")
  double[] cameraSpacePose = LimelightHelpers.getTargetPose_CameraSpace("limelight");
  double targetYaw = cameraSpacePose[4];

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    //double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    // double targetingAngularVelocity = targetYaw * kP;


    // // convert to radians per second for our drive method
    // targetingAngularVelocity *= Constants.LimelightConstants.kMaxAngularSpeed;

    // //invert since tx is positive when the target is to the right of the crosshair
    // targetingAngularVelocity *= -1.0;

    double targetingAngularVelocity = pidController.calculate(targetYaw, 0) * -0.05;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional() {    
    // double kP = -.03;
    // double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    // targetingForwardSpeed *= Constants.LimelightConstants.kMaxSpeed;
    // targetingForwardSpeed *= -1.0;
    double targetingForwardSpeed = pidController.calculate(LimelightHelpers.getTY("limelight"), 0) * -0.02;
    return targetingForwardSpeed;
  }


  double limelight_strafe_proportional() {
    // double kP = -0.01; // tune this
    // double tx = LimelightHelpers.getTX("limelight");
    // double strafeSpeed = tx * kP;
    // strafeSpeed *= Constants.LimelightConstants.kMaxSpeed;
    // strafeSpeed *= -1.0;

    double strafeSpeed = pidController.calculate(LimelightHelpers.getTX("limelight"), 0) * -0.02;

    return strafeSpeed;
  }
  

  public RobotContainer() {
 
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), driverXbox::getLeftY, driverXbox::getLeftX);
    driveAngularVelocity = driveAngularVelocity.withControllerRotationAxis(driverXbox::getRightX);
    driveAngularVelocity = driveAngularVelocity.deadband(OperatorConstants.DEADBAND);
    driveAngularVelocity = driveAngularVelocity.scaleTranslation(0.1);
    driveAngularVelocity = driveAngularVelocity.allianceRelativeControl(true);
    
  

    driveDirectAngle = driveAngularVelocity.copy();
    driveDirectAngle = driveDirectAngle.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY);
    driveDirectAngle = driveDirectAngle.headingWhile(true);

    driveRobotOriented = driveAngularVelocity.copy();
    driveRobotOriented = driveRobotOriented.robotRelative(true);
    driveRobotOriented = driveRobotOriented.allianceRelativeControl(false);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

    driverXbox.a().whileTrue((Commands.runOnce(() -> driveWithLimelight()).repeatedly()));    
  }

  private void driveWithLimelight() {
    double ySpeed = limelight_strafe_proportional();
    double rot = limelight_aim_proportional();
    double xSpeed = limelight_range_proportional(); 
    Translation2d translation = new Translation2d(xSpeed, ySpeed);
    drivebase.drive(translation, rot, false);

    if (!LimelightHelpers.getTV("limelight")) {
      drivebase.drive(new Translation2d(0, 0), 0, false);
      return;
    }
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
