// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setTurretAngle_cmd extends Command {
  public SS_Turret turret;
  private PIDController rController = new PIDController(7, 0, 0); //3.2 0.2 0.002 // 1.7, 1.2, 0
  private double pidSpeed;
  private double flipCorrection;
  private double tempTargetAngle;
  private double turretAngle;
  private double rotationsDifference;
  /** Moves the turret to a set target angle in degrees. */
  public setTurretAngle_cmd(SS_Turret turret, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    targetAngle = tempTargetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.reset();
    rController.setSetpoint(0);
    rController.setTolerance(0.005); // 0.01
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretAngle = 360*turret.getTurretRotation();
    if ((tempTargetAngle + flipCorrection > 360*turret.leftMaximum) || (tempTargetAngle + flipCorrection < 360*turret.rightMaximum)) {
      if (tempTargetAngle + flipCorrection >= 0) {
        flipCorrection -= 360;
      } else {
        flipCorrection += 360;
      }
    } else {
      rotationsDifference = (turretAngle - tempTargetAngle - flipCorrection)/360;
      pidSpeed = rController.calculate(rotationsDifference);
      if (!rController.atSetpoint()) {
        turret.setRawSpeed(pidSpeed); //This is the turret's speed
      } else {
        turret.setRawSpeed(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rController.atSetpoint()){
      return true;
    } else {
      return false;
    }
  }
}
