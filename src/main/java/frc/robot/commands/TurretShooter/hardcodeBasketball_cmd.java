// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_Throat;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class hardcodeBasketball_cmd extends Command {
  //Shooting Distance while on the Depot

  SS_Shooter shooter;
  SS_Throat throat;
  SS_Hopper hopper;
  /** Creates a new AutoHardcodedScore_cmd. */
  public hardcodeBasketball_cmd(SS_Shooter ss_shooter, SS_Throat ss_throat, SS_Hopper ss_hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_shooter);
    this.shooter = ss_shooter;
    this.throat = ss_throat;
    this.hopper = ss_hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(1);

    shooter.hoodLifter.setPosition(0.3);

    throat.throatMotor.set(0.7);
    hopper.hotDogRollersOn();
    hopper.carWashOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.carWashOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
