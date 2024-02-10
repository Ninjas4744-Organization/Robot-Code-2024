// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import java.util.function.Supplier;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.Swerve;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ImageProcCommand extends SequentialCommandGroup {
//   Swerve _swerve;
//   Pose2d _newPose;
//   PathPlannerTrajectory _currentPath;
//   Supplier<Pose2d> _currentPose;
//   ProfiledPIDController thetaController =
//         new ProfiledPIDController(
//             Constants.AutoConstants.kPThetaController,
//             0,
//             0,
//             Constants.AutoConstants.kThetaControllerConstraints);
    
//   /** Creates a new ImageProcCommand. */
//   public ImageProcCommand(Swerve swerve,Supplier<Pose2d> currentPose) {
//     _currentPose = currentPose;

//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
//     _swerve = swerve;

  
//     _newPose = currentPose.get();
//     _currentPath = PathPlanner.generatePath(new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
//     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), false,
//     new PathPoint(_newPose.getTranslation(),_newPose.getRotation()),
//     new PathPoint(new Translation2d(Units.inchesToMeters(610.77-20),
//     Units.inchesToMeters(42.19)),new Rotation2d()));

//     addCommands(
//       new InstantCommand(()-> {
//       _swerve.resetOdometry(_newPose);
//     }),

//     new SwerveControllerCommand(
//         _currentPath ,
//         _swerve::getPose,
//         Constants.Swerve.swerveKinematics,
//         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//         thetaController,
//         _swerve::setModuleStates,
//         _swerve)
    
//     );
//   }

  
// }
