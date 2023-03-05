// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    addCommands(// IMPORTANT: THIS SHOULD BE HOVERING ABOVE THE NODE BY ABOUT 15 DEGREES
    new AlignCommand(s_Swerve, s_Arm, s_Intake, x, angle),

    new ArmCommand(s_Arm, angle),

    // Outtake Command with hardcoded time (TODO: check the appropiate number of seconds)
    new IntakeCommand(s_Arm, s_Intake, 1, false),

    // Return to docked position
    new ArmCommand(s_Arm, -40));
  }
}
