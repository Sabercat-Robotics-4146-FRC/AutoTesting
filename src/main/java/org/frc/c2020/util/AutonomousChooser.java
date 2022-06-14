package org.frc.c2020.util;

import org.frc.c2020.RobotContainer;
import org.frc.c2020.commands.FollowTrajectoryCommand;
import org.frc.c2020.commands.ShootBallCommand;
import org.frc.common.control.Trajectory;
import org.frc.common.math.RigidTransform2;
import org.frc.common.math.Rotation2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChooser {
  private final AutonomousTrajectories trajectories;

  private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

  public AutonomousChooser(AutonomousTrajectories trajectories) {
    this.trajectories = trajectories;

    autonomousModeChooser.addOption("Straight Line", AutonomousMode.STRAIGHT_LINE);
    autonomousModeChooser.addOption("Sraight Back and Shoot", AutonomousMode.STRAIGHT_BACK_SHOOT);
  }

  public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
    return autonomousModeChooser;
  }

  public Command getStraightAutoCommand(RobotContainer container) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    resetRobotPose(command, container, trajectories.getStraightAutoPartOne());

    follow(command, container, trajectories.getStraightAutoPartOne());

    return command;
  }

  public Command getStraightBackShootCommand(RobotContainer container) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    resetRobotPose(command, container, trajectories.getStraightBackAndShoot());
    shootAtTarget(command, container);
    follow(command, container, trajectories.getStraightBackAndShoot());
    return command;
  }

  public Command getCommand(RobotContainer container) {
    switch (autonomousModeChooser.getSelected()) {
      case STRAIGHT_LINE:
        return getStraightAutoCommand(container);
      case STRAIGHT_BACK_SHOOT:
        return getStraightBackShootCommand(container);
    }

    return getStraightAutoCommand(container);
  }

  private void shootAtTarget(SequentialCommandGroup command, RobotContainer container) {
    command.addCommands(new ShootBallCommand(container.getFlywheelSubsystem()));
  }

  private void follow(
      SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
    command.addCommands(
        new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
  }

  private void resetRobotPose(
      SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
    command.addCommands(
        new InstantCommand(
            () -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
    command.addCommands(
        new InstantCommand(
            () ->
                container
                    .getDrivetrainSubsystem()
                    .resetPose(
                        new RigidTransform2(
                            trajectory.calculate(0.0).getPathState().getPosition(),
                            Rotation2.ZERO))));
  }

  private enum AutonomousMode {
    STRAIGHT_LINE,
    STRAIGHT_BACK_SHOOT
  }
}