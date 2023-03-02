// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.autos.TrajectoryFollowCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AlignCommand extends CommandBase {
  /** Creates a new ScoreCommand. */
  private Swerve s_Swerve;
  private Arm s_Arm;
  private Intake s_Intake;
  private double xPose;
  private double gamePieceAngle;
  private final double cutoffXCord = 2.91;
  private boolean done;
  public AlignCommand(Swerve drive, Arm arm, Intake intake, double x, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = drive;
    s_Arm = arm;
    s_Intake = intake;
    xPose = x;
    gamePieceAngle = angle;
    addRequirements(drive, arm, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Swerve.getPose().getX() < cutoffXCord && s_Swerve.getPose().getX() >= 0 && xPose < cutoffXCord && xPose >= 0){ // For safety
    // Go to position
        new TrajectoryFollowCommand(s_Swerve, s_Swerve.getPose(), new ArrayList<Translation2d>(), new Pose2d(new Translation2d(xPose, s_Swerve.getPose().getY()), // Go to specified xPose 
          new Rotation2d(0)), false);
      
    }
    else{
      done = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done||(s_Swerve.getPose().getX()>xPose-0.02 && s_Swerve.getPose().getX()<xPose+0.02) ;
  }
}
