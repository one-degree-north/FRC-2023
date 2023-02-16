package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class VisionConstants {
        public static final String cameraName = "OV5647"; // Camera name from PhotonVision

        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.178, 0.0, 0.127), // Camera offset in meters - TODO
                        // X should be forward/back and Z should be height
                        new Rotation3d(
                                0, 0,
                                0)); // YPR offset for limelight
    }

    public static final class ArmConstants {
        // CAN IDs
        public static final int armMotorID = 30;
        public static final int armSlaveID = 31;
        public static final int armEncoderID = 50;

        // Inverts
        public static final boolean motorInvert = false;
        public static final boolean slaveInvert = true;
        public static final boolean canCoderInvert = false;

        // Offset in degrees
        public static final double angleOffset = -43.287+254;

        // Gear ratio
        public static final double gearRatio = (48/15)*100;

        // Profiled PID controller constants
        public static final double kP = 0.10369;
        public static final double kI = 0;
        public static final double kD = 0.070981;

        // Arm feedforward constants
        public static final double kS = 0.10387;
        public static final double kG = 0.21111;
        public static final double kV = 0.096933;
        public static final double kA = 0.0033986;

        // Constraints for motion profiling
        public static final double velConstraint = 90;
        public static final double accelConstraint = 100;
    }

    public static final class IntakeConstants {
        public static final int intakeID = 40; 
        public static final double speed = 0.4;
        public static final boolean inverted = false;
    }


    public static final class SwerveConstants {

        /* Drivetrain Constants */
        public static final double trackWidth = 0.64;
        public static final double wheelBase = 0.64;
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.55 / 1.0); // 6.55:1 ?
        public static final double angleGearRatio = (72.0 / 14.0) * (24.0 / 12.0); // Mystery gear ratio... 72/7:1 (WYSI)

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true; 

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true; 

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 20;
            public static final double angleOffset = 305.06;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 23;
            public static final double angleOffset = 322.20;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final double angleOffset = 173.67;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 22;
            public static final double angleOffset = 321.15;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {

        // Be very when increasing max speed
        public static final double kMaxSpeedMetersPerSecond = 0.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*5/4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI*5/4;
    
        public static final double kPXController = 0.6;
        public static final double kPYController = 0.6;
        public static final double kPThetaController = 0.8;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
