// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class manualHood_cmd extends Command {
  public SS_Shooter shooter;

  private CommandXboxController joystick2;

  private double hoodPosition = 0;

  /** Creates a new lowerHood. */
  public manualHood_cmd(SS_Shooter ss_shooter, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_shooter);
    this.shooter = ss_shooter;
    this.joystick2 = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (-joystick2.getLeftY() > 0.5 && hoodPosition < 0.8) {
      hoodPosition = hoodPosition + 0.01;
    } else if (-joystick2.getLeftY() < -0.5 && hoodPosition > 0){
      hoodPosition = hoodPosition - 0.01;
    }
    shooter.hoodLifter.setPosition(hoodPosition);
    SmartDashboard.putNumber("hoodposition", hoodPosition);
    SmartDashboard.putNumber("rightX", joystick2.getLeftY());
    shooter.leftShooter.set(0);
    shooter.rightShooter.set(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
