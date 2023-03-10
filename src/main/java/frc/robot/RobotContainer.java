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
  private final double rateLimit = 5;
  boolean fieldRelative = true;
  boolean openLoop = true;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger zeroGyro = new Trigger(() -> driver.getPOV() == 0);
  private final Trigger zeroArm = new Trigger(() -> driver.getPOV() == 180);
  private final Trigger scoreLow = new Trigger(() -> driver.getPOV() == 90);
  private final Trigger scoreHigh = new Trigger(() -> driver.getPOV() == 270);
  private final JoystickButton armHighScore = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton armLowScore = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton armHighIntake = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton armLowIntake = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  


  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Arm s_Arm = new Arm();
  public final Intake s_Intake = new Intake();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /* Command Stuff */

  // This position is HOVERING SLIGHTLY ABOVE THE INTAKE LOW POSITION. 
  private final double DOCKED_POSITION = -43;

  private final double INTAKE_HIGH = 45; //Need to Check

  // This position is as low to the floor as the intake can get within arm constraints. 
  private final double INTAKE_LOW = 220; 

  private final double OUTTAKE_MID = 162; //Need to  double Check
  private final double OUTTAKE_NEAR = 1.85; //Need to Check
  private final double OUTTAKE_FAR = 2.00; //Need to Check
  private final double OUTTAKE_LOW = 220; //Need to double Check

  // Commands

  

  
  


  //WORK IN PROGRESS (abandoned)
  // private Command getHighIntakeCommand(double distanceFromWall, double cutoffXCord, double intakingAngle) {
  //   if (s_Swerve.getPose().getX() < cutoffXCord)
  //   return new SequentialCommandGroup(s_Swerve.getGoToPoseCommand(true, new Pose2d(
  //     new Translation2d(-distanceFromWall, 0), 
  //   new Rotation2d(0))),

  //   );
  // }


  // "Close Side" means the side closest to the other alliance's loading zone, while "Far Side" means the side furthest.

  // Naming syntax: GP# (game pieces) C (omit if no charge station/climb) _ CS/MS/FS (close/middle/far side)

  // private Command GP2_C_CS = new SequentialCommandGroup( // 2 game piece with climb from close side
  //   // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
  //   new ArmCommand(s_Arm, OUTTAKE_MID),
  //   new IntakeCommand(s_Arm, s_Intake, 1, false),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),
  //   new PathPlannerFollowCommand(s_Swerve, "Score1ToGamePiece1"), 
    
  //   new ArmCommand(s_Arm, INTAKE_LOW),
  //   // TODO: TUNE INTAKE COMMAND SECONDS
  //   new IntakeCommand(s_Arm, s_Intake, 1, true),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),
  //   new PathPlannerFollowCommand(s_Swerve, "GamePiece1ToScore2"),

  //   // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
  //   new ArmCommand(s_Arm, OUTTAKE_MID),
  //   new IntakeCommand(s_Arm, s_Intake, 1, false),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),

  //   new PathPlannerFollowCommand(s_Swerve, "Score2ToChargingStation")
  // );

  private Command GP3_CS = new SequentialCommandGroup( // 3 game piece with climb from close side
    // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
    new ArmCommand(s_Arm, OUTTAKE_MID),
    new IntakeCommand(s_Arm, s_Intake, 1, false),
    new ArmCommand(s_Arm, DOCKED_POSITION),
    new PathPlannerFollowCommand(s_Swerve, "Score1ToGamePiece1"), 
    
    new ArmCommand(s_Arm, INTAKE_LOW),
    // TODO: TUNE INTAKE COMMAND SECONDS
    new IntakeCommand(s_Arm, s_Intake, 1, true),
    new ArmCommand(s_Arm, DOCKED_POSITION)
    // new PathPlannerFollowCommand(s_Swerve, "GamePiece1ToScore2"),

    // // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
    // new ArmCommand(s_Arm, OUTTAKE_MID),
    // new IntakeCommand(s_Arm, s_Intake, 1, false),
    // new ArmCommand(s_Arm, DOCKED_POSITION),
    // new PathPlannerFollowCommand(s_Swerve, "Score2ToGamePiece2"),
    // new IntakeCommand(s_Arm, s_Intake, 1, true),
    // new ArmCommand(s_Arm, DOCKED_POSITION)
  );

  // private Command GP2_C_FS = new SequentialCommandGroup( // 2 game piece with climb from far side
  //   // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
  //   new ArmCommand(s_Arm, OUTTAKE_MID),
  //   new IntakeCommand(s_Arm, s_Intake, 1, false),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),
  //   new PathPlannerFollowCommand(s_Swerve, "Score9ToGamePiece4"), 
    
  //   new ArmCommand(s_Arm, INTAKE_LOW),
  //   // TODO: TUNE INTAKE COMMAND SECONDS
  //   new IntakeCommand(s_Arm, s_Intake, 1, true),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),
  //   new PathPlannerFollowCommand(s_Swerve, "GamePiece4ToScore8"),

  //   // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
  //   new ArmCommand(s_Arm, OUTTAKE_MID),
  //   new IntakeCommand(s_Arm, s_Intake, 1, false),
  //   new ArmCommand(s_Arm, DOCKED_POSITION),
  //   new PathPlannerFollowCommand(s_Swerve, "Score8ToChargingStation")
  // );

  private Command GP3_FS = new SequentialCommandGroup( // 3 game piece  from far side
    // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
    new ArmCommand(s_Arm, OUTTAKE_MID),
    new IntakeCommand(s_Arm, s_Intake, 1, false),
    new ArmCommand(s_Arm, DOCKED_POSITION),
    new PathPlannerFollowCommand(s_Swerve, "Score9ToGamePiece4"), 
    
    new ArmCommand(s_Arm, INTAKE_LOW),
    // TODO: TUNE INTAKE COMMAND SECONDS
    new IntakeCommand(s_Arm, s_Intake, 1, true),
    new ArmCommand(s_Arm, DOCKED_POSITION)
    // new PathPlannerFollowCommand(s_Swerve, "GamePiece4ToScore8"),

    // // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
    // new ArmCommand(s_Arm, OUTTAKE_MID),
    // new IntakeCommand(s_Arm, s_Intake, 1, false),
    // new ArmCommand(s_Arm, DOCKED_POSITION),
    // new PathPlannerFollowCommand(s_Swerve, "Score8toGamePiece3"),
    // new IntakeCommand(s_Arm, s_Intake, 1, true),
    // new ArmCommand(s_Arm, DOCKED_POSITION)
  );



  private Command MD = new SequentialCommandGroup( // Score and Charge Station
    // TODO: TUNE X POSE (X POSITION ON PATHPLANNER THAT WILL LET US SCORE)
    new ScoreCommand(s_Swerve, s_Arm, s_Intake,1.8, OUTTAKE_MID),
    new ArmCommand(s_Arm, DOCKED_POSITION),
    new PathPlannerFollowCommand(s_Swerve, "Charging_Station")
  
  );


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, rateLimit));

    // ADD ALL AUTONS HERE
    // m_chooser.addOption("GP2C_CS", GP2_C_CS);
    m_chooser.addOption("GP3_CS", GP3_CS);
    // m_chooser.addOption("GP2C_FS", GP2_C_FS);
    m_chooser.addOption("GP3_FS", GP3_FS);
    m_chooser.addOption("Mid", MD);
    m_chooser.addOption("testing124", new PathPlannerFollowCommandOdo(s_Swerve, "testing124"));
    m_chooser.addOption("Test Drive", new PathPlannerFollowCommand(s_Swerve, "New New Path"));
    m_chooser.addOption("Middle Basic Auton", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.intake()), new WaitCommand(0.2), new InstantCommand(() -> s_Intake.stop()), new ArmCommand(s_Arm, OUTTAKE_MID), new IntakeCommand(s_Arm, s_Intake, 1, false), new ArmCommand(s_Arm, DOCKED_POSITION), new PathPlannerFollowCommandOdo(s_Swerve, "Middle Out of Community")));



    // Naming syntax: GP# (game pieces) C (omit if no charge station/climb) _ CS/MS/FS (close/middle/far side)
    m_chooser.addOption("Basic Auton", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.intake()), new WaitCommand(0.2), new InstantCommand(() -> s_Intake.stop()), new ArmCommand(s_Arm, OUTTAKE_MID), new IntakeCommand(s_Arm, s_Intake, 1, false), new ArmCommand(s_Arm, DOCKED_POSITION), new PathPlannerFollowCommandOdo(s_Swerve, "Out of Community")));

    m_chooser.setDefaultOption("Just score", new SequentialCommandGroup(new InstantCommand(() -> s_Intake.intake()), new WaitCommand(0.2), new InstantCommand(() -> s_Intake.stop()), new ArmCommand(s_Arm, OUTTAKE_MID), new IntakeCommand(s_Arm, s_Intake, 1, false), new ArmCommand(s_Arm, DOCKED_POSITION)));

    // ShuffleBoard auto selection options
    SmartDashboard.putData("Auto choices", m_chooser);

    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    /* Driver Buttons - MANUAL CONTROL FOR TESTING */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroArm.onTrue(new InstantCommand(() -> s_Arm.setGoal(DOCKED_POSITION)));


    armHighScore.onTrue(new InstantCommand(() -> s_Arm.setGoal(OUTTAKE_MID)));
    armLowScore.onTrue(new InstantCommand(() -> s_Arm.setGoal(OUTTAKE_LOW)));
    armHighIntake.onTrue(new InstantCommand(() -> s_Arm.setGoal(INTAKE_HIGH)));
    armLowIntake.onTrue(new InstantCommand(() -> s_Arm.setGoal(INTAKE_LOW)));

    intakeIn.onTrue(new IntakeCommand(s_Arm, s_Intake, 1, true));
    intakeOut.onTrue(new IntakeCommand(s_Arm, s_Intake, 1, false));

    // scoreLow.onTrue(new ScoreCommand(s_Swerve, s_Arm, s_Intake,2.0, OUTTAKE_LOW));
    // scoreLow.onTrue(new ScoreCommand(s_Swerve, s_Arm, s_Intake,1.8, OUTTAKE_MID));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(new InstantCommand(() -> s_Arm.setCurrentPosToGoal()), m_chooser.getSelected());
  }
}
