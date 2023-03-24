// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class BalanceCommand extends CommandBase {
  private boolean m_reverse;
  private Swerve s_Swerve;

  private PIDController m_controller;

  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0;
  private double tolerance = 2.0;
  private double maxOutput = 0.1;

  private double angle;
  private double output;


  /** Creates a new BalanceCommand. */
  public BalanceCommand(boolean reverse, Swerve swerve) {
    this.s_Swerve = swerve;
    this.m_reverse = reverse;
    addRequirements(s_Swerve);

    m_controller = new PIDController(kP, kI, kD);
    m_controller.setTolerance(tolerance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = s_Swerve.getPitch();
    output = (!m_reverse) ? MathUtil.clamp(m_controller.calculate(angle), -maxOutput, maxOutput) 
    : -1 * MathUtil.clamp(m_controller.calculate(angle), -maxOutput, maxOutput);
    s_Swerve.drive(new Translation2d(output, 0), 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
