// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

//Test Commit

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final double rateLimit = 2;
  boolean fieldRelative = true;
  boolean openLoop = true;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger zeroGyro = new Trigger(() -> driver.getPOV() == 0);
  private final Trigger zeroArm = new Trigger(() -> driver.getPOV() == 180);
  private final Trigger intake = new Trigger(() -> driver.getPOV() == 90);
  private final Trigger score = new Trigger(() -> driver.getPOV() == 270);
  private final JoystickButton armHighScore = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton armLowScore = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton armHighIntake = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton armLowIntake = new JoystickButton(driver, XboxController.Button.kA.value);
  private final Trigger intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  


  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Arm s_Arm = new Arm();
  public final Intake s_Intake = new Intake();
  public final PoseEstimate position = new PoseEstimate();
  SendableChooser<String> m_chooser = new SendableChooser<>();

  /* Command Stuff */

  private final double DOCKED_POSITION = -50;
  private final double INTAKE_HIGH = 25; //TO DO
  private final double INTAKE_LOW = -35;

  private final double OUTTAKE_MID = 195; // TO DO
  private final double OUTTAKE_LOW = 205; // TO DO




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, rateLimit));

    m_chooser.addOption("New Path", "New Path");

    // ShuffleBoard auto selection options
    SmartDashboard.putData("Auto choices", m_chooser);

    // Configure the button bindings
    configureButtonBindings();
  }
  // Commands

  private Command getIntakeCommand(double seconds, boolean isIntaking) {
    if (isIntaking) 
    return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(() -> s_Intake.intake()), new InstantCommand(() -> s_Arm.setGoal(s_Arm.getGoal()-15))), 
      new WaitCommand(seconds), new InstantCommand(() -> s_Intake.stop()));

    else
    return new SequentialCommandGroup(new ParallelCommandGroup(new InstantCommand(() -> s_Intake.outtake()), new InstantCommand(() -> s_Arm.setGoal(s_Arm.getGoal()+15))), 
      new WaitCommand(seconds), new InstantCommand(() -> s_Intake.stop()));

    
  }

  private Command getScoreGamePieceCommand(double xPose, double gamePieceAngle) {
    double cutoffXCord = 2.91;
    if (s_Swerve.getPose().getX() < cutoffXCord)
    return new SequentialCommandGroup(s_Swerve.getGoToPoseCommand(false, 
    new Pose2d(new Translation2d(xPose, s_Swerve.getPose().getTranslation().getY()), 
    new Rotation2d(0))), 
    new ArmCommand(s_Arm, gamePieceAngle),
    getIntakeCommand(1.5, false),
    new ArmCommand(s_Arm, DOCKED_POSITION)
    );

    else return new InstantCommand();
  }


  //WORK IN PROGRESS
  // private Command getHighIntakeCommand(double distanceFromWall, double cutoffXCord, double intakingAngle) {
  //   if (s_Swerve.getPose().getX() < cutoffXCord)
  //   return new SequentialCommandGroup(s_Swerve.getGoToPoseCommand(true, new Pose2d(
  //     new Translation2d(-distanceFromWall, 0), 
  //   new Rotation2d(0))),

  //   );
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroArm.onTrue(new InstantCommand(() -> s_Arm.setGoal(DOCKED_POSITION)));

    intake.onTrue(getIntakeCommand(2, true));
    score.onTrue(getIntakeCommand(2, false));

    armHighScore.onTrue(new InstantCommand(() -> s_Arm.setGoal(OUTTAKE_MID)));
    armLowScore.onTrue(new InstantCommand(() -> s_Arm.setGoal(OUTTAKE_LOW)));
    armHighIntake.onTrue(new InstantCommand(() -> s_Arm.setGoal(INTAKE_HIGH)));
    armLowIntake.onTrue(new InstantCommand(() -> s_Arm.setGoal(INTAKE_LOW)));

    intakeIn.onTrue(new InstantCommand(() -> s_Intake.intake()));
    intakeIn.onFalse(new InstantCommand(() -> s_Intake.stop()));
    intakeOut.onTrue(new InstantCommand(() -> s_Intake.outtake()));
    intakeOut.onFalse(new InstantCommand(() -> s_Intake.stop()));




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PathPlannerFollowCommand(s_Swerve, m_chooser.getSelected());
  }
}
