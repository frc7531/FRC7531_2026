// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Turret;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class manualTurret extends Command {
  public SS_Turret turret;
  public double speed;
  private double rightX;
  private CommandXboxController joystick2;
  public manualTurret(SS_Turret ss_turret, CommandXboxController controller) {
    addRequirements(ss_turret);
    this.turret = ss_turret;
    this.joystick2 = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightX = -joystick2.getRightX();
    if (Math.abs(rightX) > 0.1) { // Stick deadzone
    speed = rightX * 0.2;
    } else {
    speed = 0;
    }
    SmartDashboard.putNumber("manualSpeed", speed);

    if ((turret.getTurretRotation() > turret.leftMaximum) && (speed > 0)) {
      turret.setRawSpeed(0);
      return;
    } else if ((turret.getTurretRotation() < turret.rightMaximum) && (speed < 0)) {
      turret.setRawSpeed(0);
      return;
    } else {
      turret.setRawSpeed(speed);
    }
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
