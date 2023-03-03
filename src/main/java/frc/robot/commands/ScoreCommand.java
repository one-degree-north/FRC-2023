// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.TrajectoryFollowCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommand extends SequentialCommandGroup {
  /** Creates a new ScoreCommand. */
  public ScoreCommand(Swerve s_Swerve, Arm s_Arm,  Intake s_Intake, double x, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    if (s_Swerve.getPose().getX() < 2.91 && s_Swerve.getPose().getX() >= 0 && x < 2.91 && x >= 0){
      addCommands(// IMPORTANT: THIS SHOULD BE HOVERING ABOVE THE NODE BY ABOUT 15 DEGREES
      new TrajectoryFollowCommand(s_Swerve, new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), // Go to specified xPose 
      new Rotation2d(3.1)), new ArrayList<Translation2d>(), new Pose2d(new Translation2d(x, s_Swerve.getPose().getY()), // Go to specified xPose 
          new Rotation2d(3.1)), true),
      new ArmCommand(s_Arm, angle),

      // Outtake Command with hardcoded time (TODO: check the appropiate number of seconds)
      new IntakeCommand(s_Arm, s_Intake, 1, false),

      // Return to docked position
      new ArmCommand(s_Arm, -42));
    }
    else if(s_Swerve.getPose().getX() > (16.5-2.91) && s_Swerve.getPose().getX() <=16.5 && x <2.91 && x >= 0){
      addCommands(// IMPORTANT: THIS SHOULD BE HOVERING ABOVE THE NODE BY ABOUT 15 DEGREES
      new TrajectoryFollowCommand(s_Swerve, new Pose2d(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), // Go to specified xPose 
      new Rotation2d(0)), new ArrayList<Translation2d>(), new Pose2d(new Translation2d(16.5-x, s_Swerve.getPose().getY()), // Go to specified xPose 
          new Rotation2d(0)), true),

      new ArmCommand(s_Arm, angle),

      // Outtake Command with hardcoded time (TODO: check the appropiate number of seconds)
      new IntakeCommand(s_Arm, s_Intake, 1, false),

      // Return to docked position
      new ArmCommand(s_Arm, -42));
    }
  }
}
