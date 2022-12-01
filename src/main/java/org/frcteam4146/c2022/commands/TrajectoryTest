package org.frcteam4146.c2022.commands.auto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

import org.frcteam4146.c2022.commands.FollowTrajectoryCommand;
import org.frcteam4146.c2022.subsystems.DrivetrainSubsystem;
import org.frcteam4146.common.control.SimplePathBuilder;
import org.frcteam4146.common.math.Rotation2;
import org.frcteam4146.common.math.Vector2;

public class TrajectoryTest extends SequentialCommandGroup {

  public TrajectoryTest(DrivetrainSubsystem drivetrain) {
    Trajectory t = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 1), new Translation2d(-2, -1)),
      new Pose2d(3, 0, new Rotation2d(0)), 
      new TrajectoryConfig(100, 100)
    );



    RamseteCommand ramseteCommand = new RamseteCommand(
      t, 
      drivetrain::getPose, 
      new RamseteController(2, .7), 
      new SimpleMotorFeedforward(.22, 1.98, 0.2), 
      new DifferentialDriveKinematics(.7), 
      drivetrain::g, 
      leftController, 
      rightController, 
      outputVolts, 
      requirements
    );
  }
}
