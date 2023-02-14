package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TrajectoryFollowCommand extends SequentialCommandGroup {
    public TrajectoryFollowCommand(Swerve s_Swerve, Pose2d startPose, List<Translation2d> interiorWaypoints, Pose2d endPose, boolean odometryOnly){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.SwerveConstants.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory traj =
            TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                endPose,
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand;
        if (!odometryOnly) {

            swerveControllerCommand =
                new SwerveControllerCommand(
                    traj,
                    s_Swerve::getPose,
                    Constants.SwerveConstants.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
            addCommands(new InstantCommand(() -> {
            s_Swerve.resetOdometry(traj.getInitialPose());
          }, s_Swerve), new InstantCommand(() -> {
            s_Swerve.resetPose(traj.getInitialPose());
          }, s_Swerve),
              swerveControllerCommand);

        } else {

            swerveControllerCommand =
                new SwerveControllerCommand(
                    traj,
                    s_Swerve::getOdometryPose,
                    Constants.SwerveConstants.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
            addCommands(new InstantCommand(() -> {
            s_Swerve.resetOdometry(traj.getInitialPose());
          }, s_Swerve),
              swerveControllerCommand);

        }
    }
}