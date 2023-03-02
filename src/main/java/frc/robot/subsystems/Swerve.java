package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.SwerveModule;
import frc.robot.autos.TrajectoryFollowCommand;
import frc.robot.Constants;
import frc.robot.PoseEstimate;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public PoseEstimate poseEstimateHelper;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Field2d field2d;

    public Swerve() {
        poseEstimateHelper = new PoseEstimate();
        field2d = new Field2d();
        gyro = new AHRS(SPI.Port.kMXP);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(), swerveOdometry.getPoseMeters());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.3,0.3,0.3));

        for(SwerveModule mod : mSwerveMods){
            System.out.println("CANcoder on Module " + mod.moduleNumber + " took " + mod.CANcoderInitTime + " ms to be ready.");
        }

        SmartDashboard.putData(field2d);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setRobotPose2d(Pose2d pose2d) {
        field2d.setRobotPose(pose2d);
    }

    public void setFieldTrajectory(String name, Trajectory traj) {
        field2d.getObject(name).setTrajectory(traj);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometryWithNewRotation(Pose2d pose, Rotation2d initalRotation) {
        Pose2d newPose2d = new Pose2d(pose.getTranslation(), initalRotation);
        swerveOdometry.resetPosition(getYaw(), getPositions(), newPose2d);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }
    
    public void calibrateAndResetGyro() {
        gyro.calibrate();
        int counter = 0; boolean timedOut = false;
        while (gyro.isCalibrating()){
            if (counter > 4) {
                timedOut = true;
                break;
            }
            Timer.delay(0.5);
            counter++;
        }
        if (!timedOut) {
            System.out.println("Gyro calibration done!");
            resetGyro();
        }
        else {
            System.out.println("Gyro calibration timed out!");
        }
    }

    @Override
    public void periodic(){

        swerveOdometry.update(getYaw(), getPositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Gyro Data", getYaw().getDegrees());
        
        updateOdometry();
        
    }

    public void updateOdometry() {
        poseEstimator.update(getYaw(), getPositions());
        field2d.getObject("Odometry").setPose(swerveOdometry.getPoseMeters());

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        Pair<Pose3d, Double> result =
                poseEstimateHelper.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        var camPose = result.getFirst();
        var camPoseObsTime = result.getSecond();
        if (camPose != null) {
                poseEstimator.addVisionMeasurement(camPose.toPose2d(), camPoseObsTime);
                field2d.getObject("Vision position").setPose(camPose.toPose2d());
        }
        field2d.setRobotPose(getPose());
    }

    // Use odometryOnly mode if target pose is relative to startingPose instead of the entire field
    public Command getGoToPoseCommand(boolean odometryOnly, Pose2d targetPose) {
        if (!odometryOnly) return new TrajectoryFollowCommand(this, getPose(), new ArrayList<Translation2d>(), targetPose, false);
        else return new TrajectoryFollowCommand(this, new Pose2d(0, 0, getYaw()), new ArrayList<Translation2d>(), targetPose, false);
    }

}