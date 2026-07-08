// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.joystickInversion;
import frc.robot.commands.Hang.HangLevel1_cmd;
import frc.robot.commands.Hang.HangReturnManual_cmd;
import frc.robot.commands.Hang.HangReturn_cmd;
import frc.robot.commands.Hang.HangUpManual_cmd;
import frc.robot.commands.Hang.HangerDefault_cmd;
import frc.robot.commands.Hang.alignTower; 
import frc.robot.commands.Hopper.hopperDefault_cmd;
import frc.robot.commands.Hopper.rollersForwardManual_cmd;
//import frc.robot.commands.Hopper.rollersOff_cmd;
//import frc.robot.commands.Hopper.rollersOn_cmd;
import frc.robot.commands.Hopper.rollersReverseManual_cmd;
import frc.robot.commands.Intake.AutoIntakeOff_cmd;
import frc.robot.commands.Intake.AutoIntake_cmd;
import frc.robot.commands.Intake.foldIntake_cmd;
import frc.robot.commands.Intake.intakeToggle_cmd;
import frc.robot.commands.Intake.manualFoldIntake_cmd;
import frc.robot.commands.Intake.manualUnfoldIntake_cmd;
import frc.robot.commands.Intake.outakeToggle_cmd;
//import frc.robot.commands.Hopper.rollersOff_cmd;
//import frc.robot.commands.Hopper.rollersOn_cmd;
import frc.robot.commands.Intake.unfoldIntake_cmd;
import frc.robot.commands.Throat.AutoThroatHang_cmd;
import frc.robot.commands.Throat.AutoThroatHuman_cmd;
import frc.robot.commands.Throat.AutoThroatOff_cmd;
import frc.robot.commands.Throat.AutoThroat_cmd;
import frc.robot.commands.Throat.startThroat;
import frc.robot.commands.Throat.stopThroat;
import frc.robot.commands.TurretShooter.AutoHardcodeDepotClose_cmd;
import frc.robot.commands.TurretShooter.AutoHardcodeDepot_cmd;
import frc.robot.commands.TurretShooter.AutoHardcodeHuman_cmd;
import frc.robot.commands.TurretShooter.AutoRev_cmd;
import frc.robot.commands.TurretShooter.AutoShootOff_cmd;
import frc.robot.commands.TurretShooter.AutoShoot_cmd;
import frc.robot.commands.TurretShooter.aimTurretToTarget;
import frc.robot.commands.TurretShooter.autoAimTurretToTarget;
import frc.robot.commands.TurretShooter.fireShooter;
import frc.robot.commands.TurretShooter.hardcodeBasketball_cmd;
import frc.robot.commands.TurretShooter.lobShooter;
import frc.robot.commands.TurretShooter.manualHood_cmd;
import frc.robot.commands.TurretShooter.manualShooter;
import frc.robot.commands.TurretShooter.manualTurret;
import frc.robot.commands.TurretShooter.stopTurret;
//import frc.robot.commands.TurretShooter.stopShooter; //someone needs to explain this
import frc.robot.commands.TurretShooter.lowerHood;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Intake;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_Throat;
import frc.robot.subsystems.SS_Turret;
import frc.robot.subsystems.SS_Hanger;
import frc.robot.subsystems.SS_Vision;
import frc.robot.subsystems.SS_Drivetrain.ShootMode;
import frc.robot.commands.TurretShooter.setTurretAngle_cmd;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final SS_Drivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SS_Shooter shooter = new SS_Shooter();
    public final SS_Turret turret = new SS_Turret();
    public final SS_Throat throat = new SS_Throat();
    private final SS_Intake intake = new SS_Intake();
    private final SS_Hopper hopper = new SS_Hopper();
    private final SS_Hanger hanger = new SS_Hanger();
    private final SS_Vision vision = new SS_Vision();

    public aimTurretToTarget aimCommand = new aimTurretToTarget(drivetrain, turret);
    public alignTower alignTowerCommand = new alignTower(drivetrain);
    public manualTurret manualTurret = new manualTurret(turret);
    public stopTurret stopCommand = new stopTurret(turret);
    public manualShooter shootCommand = new manualShooter(shooter);
    public stopThroat stopThroatCommand = new stopThroat(throat, hopper);
    public startThroat startThroatCommand = new startThroat(throat, hopper);
    public intakeToggle_cmd intakeToggle = new intakeToggle_cmd(intake);
    public outakeToggle_cmd outakeToggle = new outakeToggle_cmd(intake);
    public setTurretAngle_cmd setTurretAngle = new setTurretAngle_cmd(turret, 180);
    //public rollersOn_cmd hotdogOn = new rollersOn_cmd(hopper);
    //public rollersOn_cmd hotdogOnNoTimer = new rollersOn_cmd(hopper);
    public foldIntake_cmd pivotUp = new foldIntake_cmd(intake);
    public unfoldIntake_cmd pivotDown = new unfoldIntake_cmd(intake);
    //public rollersOff_cmd hotdogOff = new rollersOff_cmd(hopper);
    public fireShooter autoShooterCommand = new fireShooter(shooter, drivetrain);
    public manualShooter manualShooterCommand = new manualShooter(shooter);
    public lobShooter lobShooterCommand = new lobShooter(shooter, drivetrain, turret);
    public Command fireShooterCommand = Commands.either(
        autoShooterCommand,
        manualShooterCommand,
        () -> (turret.mode == SS_Turret.ShootMode.AUTO)
    );
    public Command shootingCommand = Commands.either(
        fireShooterCommand,
        lobShooterCommand,
        () -> (drivetrain.mode == SS_Drivetrain.ShootMode.SCORE));
    public lowerHood lowerHoodCommand = new lowerHood(shooter);
    public manualFoldIntake_cmd manualPivotUp = new manualFoldIntake_cmd(intake);
    public manualUnfoldIntake_cmd manualPivotDown = new manualUnfoldIntake_cmd(intake);
    public manualHood_cmd manualHood = new manualHood_cmd(shooter);
    public hopperDefault_cmd hopperDefault = new hopperDefault_cmd(hopper);
    public HangerDefault_cmd hangerDefault = new HangerDefault_cmd(hanger);
   
    public rollersForwardManual_cmd manualRollersForward = new rollersForwardManual_cmd(hopper);
    public rollersReverseManual_cmd manualRollersReverse = new rollersReverseManual_cmd(hopper);
    public SequentialCommandGroup hangLevel1Auto = new HangLevel1_cmd(hanger).andThen(new HangReturn_cmd(hanger));
    public HangLevel1_cmd hangUp = new HangLevel1_cmd(hanger);
    public HangReturn_cmd hangReturn = new HangReturn_cmd(hanger);
    public HangLevel1_cmd hangeLevel1Manual = new HangLevel1_cmd(hanger);
    public HangReturnManual_cmd hangReturnManual = new HangReturnManual_cmd(hanger);
    public HangUpManual_cmd hangupMan = new HangUpManual_cmd(hanger);
    public hardcodeBasketball_cmd shootBasket = new hardcodeBasketball_cmd(shooter, throat, hopper);

    public AutoHardcodeDepot_cmd autoDepot = new AutoHardcodeDepot_cmd(turret, shooter, drivetrain);
    public AutoHardcodeHuman_cmd autoHuman = new AutoHardcodeHuman_cmd(turret, shooter, drivetrain);
    public AutoHardcodeDepotClose_cmd autoDepotClose = new AutoHardcodeDepotClose_cmd(turret, shooter, drivetrain);

    // public ConditionalCommand toggleDepot = new ConditionalCommand(
    //     drivetrain.run(() -> {drivetrain.neutralTarget = drivetrain.depotPose;}), 
    //     drivetrain.run(() -> {drivetrain.neutralTarget = drivetrain.hubPose;}),
    //     () -> drivetrain.targetToggled
    // );
    // public ConditionalCommand toggleStation = new ConditionalCommand(
    //     drivetrain.run(() -> {drivetrain.neutralTarget = drivetrain.stationPose;}),
    //     drivetrain.run(() -> {drivetrain.neutralTarget = drivetrain.hubPose;}),
    //     () -> drivetrain.targetToggled
    // );
    public AutoIntake_cmd autoIntake = new AutoIntake_cmd(intake);
    public AutoIntakeOff_cmd autoIntakeOff = new AutoIntakeOff_cmd(intake);
    public AutoThroat_cmd autoThroat = new AutoThroat_cmd(throat, hopper, intake);
    public AutoThroatOff_cmd autoThroatOff = new AutoThroatOff_cmd(throat, hopper, intake);
    public AutoRev_cmd autoRev = new AutoRev_cmd(shooter); 
    public AutoShoot_cmd autoShoot = new AutoShoot_cmd(shooter);
    public AutoShootOff_cmd autoShootOff = new AutoShootOff_cmd(shooter);
    public autoAimTurretToTarget autoAim = new autoAimTurretToTarget(drivetrain, turret);
    public AutoHardcodeDepot_cmd autoDepotRev = new AutoHardcodeDepot_cmd(turret, shooter, drivetrain);
    public AutoHardcodeHuman_cmd autoHumanRev = new AutoHardcodeHuman_cmd(turret, shooter, drivetrain);
    public AutoThroatHuman_cmd autothroathuman = new AutoThroatHuman_cmd(throat, hopper);
    public AutoThroatHang_cmd autoThroatHang = new AutoThroatHang_cmd(throat, hopper);
    public joystickInversion joystickInversion = new joystickInversion(vision);
    


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    

    public RobotContainer() {
        // switch (drivetrain.alliance) {
        //     case Red:
        //     joystickInvert = 1;
        //     break;
        //     case Blue:
        //     joystickInvert = -1;
        //     break;
        // }


        configureBindings();

        //manualTurret.execute();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        NamedCommands.registerCommand("autoIntake_cmd", autoIntake);
        NamedCommands.registerCommand("autoIntakeOff_cmd", autoIntakeOff);
        NamedCommands.registerCommand("autoRev_cmd", autoRev);
        NamedCommands.registerCommand("autoThroat_cmd", autoThroat);
        NamedCommands.registerCommand("pivotDown_cmd", pivotDown);
        NamedCommands.registerCommand("pivotUp_cmd", pivotUp);
        NamedCommands.registerCommand("AlignTower_cmd", alignTowerCommand);
        NamedCommands.registerCommand("autoShoot_cmd", autoShoot);
        NamedCommands.registerCommand("autoShootOff", autoShootOff);
        NamedCommands.registerCommand("HangLevel1_cmd", hangUp);
        NamedCommands.registerCommand("HangReturn_cmd", hangReturn);
        NamedCommands.registerCommand("autoAimTurret_cmd", autoAim);
        NamedCommands.registerCommand("revOnDepot", autoDepotRev); 
        NamedCommands.registerCommand("revDepotClose", autoDepotClose);
        NamedCommands.registerCommand("revHumanClose", autoHumanRev);
        NamedCommands.registerCommand("autoThroatHuman", autothroathuman);
        NamedCommands.registerCommand("autoThroatHang", autoThroatHang);

        autoChooser = AutoBuilder.buildAutoChooser("Blue Depot to Neutral");
        SmartDashboard.putData("Auto Mode", autoChooser);


    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystickInversion.joystickInvert*joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystickInversion.joystickInvert*joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        turret.setDefaultCommand(manualTurret);
        shooter.setDefaultCommand(manualHood);
        throat.setDefaultCommand(stopThroatCommand);
        hanger.setDefaultCommand(hangerDefault);
        hopper.setDefaultCommand(hopperDefault);

        ///// Joystick 1 ///////////////////////////////////////////////////////////////////////////////////////////////////

        joystick.b().onTrue(joystickInversion);
        joystick.x().onTrue(pivotUp);
        joystick.a().onTrue(pivotDown);
        joystick.y().whileTrue(drivetrain.run(() -> drivetrain.pigeonCommand())); // Reset Gyro
        joystick.rightBumper().onTrue(intakeToggle);
        joystick.leftBumper().onTrue(outakeToggle);
        joystick.leftTrigger().whileTrue(manualPivotUp); //Intake in manually
        joystick.rightTrigger().whileTrue(manualPivotDown); //Intake out manually
        joystick.povRight().whileTrue(alignTowerCommand);

        ///// Joystick 2////////////////////////////////////////////////////////////////////////////////////////////////////
        
        joystick2.leftTrigger().whileTrue(shootingCommand); //shootingCommand
        joystick2.rightTrigger().whileTrue(startThroatCommand);
        joystick2.a().toggleOnTrue(aimCommand); 
        joystick2.leftBumper().whileTrue(manualRollersForward);
        joystick2.rightBumper().whileTrue(manualRollersReverse);

        joystick2.b().toggleOnTrue(startThroatCommand).toggleOnTrue(autoShoot);

        joystick2.start().whileTrue(hangupMan);
        joystick2.y().whileTrue(hangReturnManual);

        joystick2.povDown().whileTrue(autoRev);
        joystick2.povLeft().whileTrue(shootBasket);
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected(); // Run the path selected from the auto chooser
    }
};