// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;

public class PathPlannerFollowCommandOdo extends SequentialCommandGroup {

  public PathPlannerFollowCommandOdo(Swerve swerve, String pathName) {
    //Exclude ".path" from pathName
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        swerve::getOdometryPose,
        SwerveConstants.swerveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new PIDController(AutoConstants.kPThetaController, 0, 0),
        swerve::setModuleStates,
        true,
        swerve);

    // swerve.setFieldTrajectory("Trajectory", trajectory);

    // change the lambda to an external command or state it outside the runOnce function
    addCommands(new InstantCommand(() -> {
      swerve.resetOdometryWithNewRotation(trajectory.getInitialPose(), trajectory.getInitialPose().getRotation());
    }, swerve),
        swerveControllerCommand);
  }
}