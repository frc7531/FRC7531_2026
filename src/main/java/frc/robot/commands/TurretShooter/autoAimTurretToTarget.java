// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Turret;


public class autoAimTurretToTarget extends Command {
  /** Creates a new aimTurretToTarget. */
  public SS_Drivetrain drivetrain;
  public SS_Turret turret;

  PIDController rController = new PIDController(1.7, 1.2, 0); //3.2 0.2 0.002
  double pidSpeed;

  Translation2d estimatedPose;

  Translation2d targetPose;
  double targetAngle;
  double adjustedTargetAngle;

  double turretAngle;
  Translation2d turretEstimate;

  Translation2d poseDifference;
  double rotationsDifference;

  int flipCorrection = 0;

  String flipStatus = "";

  public SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric();
  public CANcoder encoder;

  public autoAimTurretToTarget(SS_Drivetrain ss_drivetrain, SS_Turret ss_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret);
    this.drivetrain = ss_drivetrain;
    this.turret = ss_turret;
    this.encoder = ss_turret.encoder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.reset();
    rController.setSetpoint(0);
    rController.setTolerance(0.01);
    targetPose = drivetrain.hubPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetPose != drivetrain.targetPose) {
      targetPose = drivetrain.targetPose;
    }

    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    turretEstimate = estimatedPose.plus(turret.getTurretPosition(drivetrain.pidgey.getRotation2d().getRadians() + Math.PI));

    poseDifference = targetPose.minus(turretEstimate);
    targetAngle = poseDifference.getAngle().getDegrees();

    turretAngle = 360*turret.getTurretRotation();

    adjustedTargetAngle = targetAngle - drivetrain.pidgey.getYaw().getValueAsDouble() - 180;

    if ((adjustedTargetAngle + flipCorrection > 360*turret.leftMaximum) || (adjustedTargetAngle + flipCorrection < 360*turret.rightMaximum)) {
      if (adjustedTargetAngle + flipCorrection >= 0) {
        flipCorrection -= 360;
      } else {
        flipCorrection += 360;
      }
      SmartDashboard.putNumber("flipCorrection", flipCorrection);
      flipStatus = "Flipping";
      turret.setRawSpeed(0);
    } else {
      rotationsDifference = (turretAngle - adjustedTargetAngle - flipCorrection)/360;
      pidSpeed = rController.calculate(rotationsDifference);
      flipStatus = "Good";
      SmartDashboard.putNumber("pidSpeed", pidSpeed);
      if (!rController.atSetpoint()) {
        turret.setRawSpeed(pidSpeed); //This is the turret's speed
      } else {
        turret.setRawSpeed(0);
      }
    }
    SmartDashboard.putNumber("targetAngle", targetAngle);
    SmartDashboard.putNumber("adjustedTarget", adjustedTargetAngle);
    SmartDashboard.putNumber("turretAngle", turretAngle);
    SmartDashboard.putNumber("rotationsDifference", rotationsDifference);
    SmartDashboard.putString("flipStatus", flipStatus);
    SmartDashboard.putNumber("correctedAngle", adjustedTargetAngle + flipCorrection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (drivetrain.PoseEstimaterGetX() >= 5.747){
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
