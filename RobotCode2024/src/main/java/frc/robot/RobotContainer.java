// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private static final double DRIVE_COEFFICIENT = 0.7;

  // final Swerve _swerve;
  private final CommandPS4Controller _driveController;
  private final Intake _intakes;
  // private final Vision _vision;
  // private final Rollers _rollers;
  // private final Trigger _autoScorer;
  // private final Trigger _autoReconfigurator;
  // private final Lift _lift;
  // ProfiledPIDController thetaController;

  Map<String, Command> _commands;

  public RobotContainer() {

    _driveController = new CommandPS4Controller(Constants.DRIVE_CONTROLLER_PORT);
    // _vision = new Vision();

    // _swerve = new Swerve(_vision::getEstimatedGlobalPose);
    _intakes = new Intake();
    // _rollers = new Rollers();
    // _lift = new Lift();

    // _autoScorer = new Trigger(() -> {
    //   Pose2d targetPose = _vision.getTagPose();
    //   Pose2d current_pos = _swerve.getLastCalculatedPosition();
    //   return Math.hypot(targetPose.getX() - current_pos.getX(), targetPose.getY() - current_pos.getY()) < 0.3
    //       && targetPose.getX() - current_pos.getX() < 0.1;
    // });

    // _autoReconfigurator = new Trigger(() -> {
    //   Pose2d targetPose = _vision.getTagPose();
    //   Pose2d current_pos = _swerve.getLastCalculatedPosition();
    //   return Math.hypot(targetPose.getX() - current_pos.getX(), targetPose.getY() - current_pos.getY()) < 1;
    // });

    // _intakes.setDefaultCommand(_intakes.defaultCommand());
    // _rollers.setDefaultCommand(_rollers.defaultCommand());
    // _lift.setDefaultCommand(_lift.defaultCommand());
    // _swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         _swerve,
    //         _vision,
    //         () -> -_driveController.getLeftY() * DRIVE_COEFFICIENT,
    //         () -> -_driveController.getLeftX() * DRIVE_COEFFICIENT,
    //         () -> -_driveController.getRightX() * DRIVE_COEFFICIENT,
    //         () -> false,
    //         () -> false).alongWith(new RunCommand(() -> LimelightHelpers.setLEDMode_ForceOff(null))));

    // NamedCommands.registerCommand("Outake", _rollers.runIntake(true).withTimeout(0.5));
    // NamedCommands.registerCommand("Intake", _rollers.runIntake(true).withTimeout(0.5));

    configureBindings();

  }

  private void configureBindings() {
    _driveController.square().onTrue(Commands.startEnd(() -> {
      _intakes.setPercent(0.15);
    }, ()->{_intakes.setPercent(0);}, _intakes));
    _driveController.circle().onTrue(Commands.startEnd(() -> {
      _intakes.setPercent(-0.15);
    }, ()->{_intakes.setPercent(0);}, _intakes));
    _driveController.R2().onTrue(_intakes.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    _driveController.L2().onTrue(_intakes.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    // _driveController.triangle().whileTrue(_rollers.runIntake(false));
    // _driveController.square().whileTrue(_rollers.runIntake(true));

    // _driveController.R2().whileTrue(new TeleopSwerve(
    //     _swerve,
    //     _vision,
    //     () -> -_driveController.getLeftY() * DRIVE_COEFFICIENT,
    //     () -> -_driveController.getLeftX() * DRIVE_COEFFICIENT,
    //     () -> -_driveController.getRightX() * DRIVE_COEFFICIENT,
    //     () -> true,
    //     () -> false).alongWith(new RunCommand(() -> {
    //       if (!isNotTarget()) {
    //         LimelightHelpers.setLEDMode_ForceOn(null);

    //       } else {
    //         LimelightHelpers.setLEDMode_ForceOff(null);
    //       }
    //     }))

    // );

    // _autoScorer.whileTrue(_rollers.runIntake(true));

    // _autoReconfigurator.whileTrue(Commands.either(Commands.parallel(
    //     _intakes.outakeConfiguration(),
    //     _lift.outakeConfiguration()),
    //     Commands.parallel(
    //         _intakes.intakeConfiguration(),
    //         _lift.intakeConfiguration()),
    //     _vision::isAmp));

  }

  // public Boolean isNotTarget() {
  //   Pose2d targetPose = _vision.getTagPose();
  //   Pose2d current_pos = _swerve.getLastCalculatedPosition();
  //   return !LimelightHelpers.getTV(null) ||
  //       Math.hypot(targetPose.getX() - current_pos.getX(), targetPose.getY() - current_pos.getY()) > 2;

  // }

  // public void periodic() {
  //   Pose2d targetPose = _vision.getTagPose();
  //   Pose2d current_pos = _swerve.getLastCalculatedPosition();
  //   SmartDashboard.putNumber("distance",
  //       Math.hypot(targetPose.getX() - current_pos.getX(), targetPose.getY() - current_pos.getY()));
  // }

  // private void resetModuleZeros() {
  //   _swerve.zeroGyro();
  //   // _swerve.resetModuleZeros();
  // }

  // public void disabledActions() {
  //   resetModuleZeros();
  //   _swerve.resetOdometry(new Pose2d());

  // }

  // public Command autoCommand() {

  //   // PathPlannerPath _path = PathPlannerPath.fromPathFile("Init");

  //   // return Commands.sequence(
  //   //     Commands.run(() -> _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose()), _swerve),
  //   //     AutoBuilder.buildAuto("basic"));

  //   return _intakes.sysIdDynamic();
  // }

}
