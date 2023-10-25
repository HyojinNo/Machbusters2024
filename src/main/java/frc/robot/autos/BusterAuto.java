package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class BusterAuto extends SequentialCommandGroup {
  private RobotContainer m_robotContainer;

  public BusterAuto(RobotContainer container, SendableChooser<String> chooser) {
    m_robotContainer = container;

    switch (chooser.getSelected()) {
      case "straight":
        addCommands(
            //  new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
            //  new WaitCommand(3),
            //  new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.2, 0), 0, false, false)),
            new WaitCommand(2),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false)));
        break;
      case "left":
        addCommands(
          new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
           new WaitCommand(3),
           new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(2.25, 0), 0, false, false)),
            new WaitCommand(2.5),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, false))

        );
        break;
      case "right":
        addCommands(
            new InstantCommand(() -> m_robotContainer.s_Intaker.push()),
            new WaitCommand(3),
            new InstantCommand((() -> m_robotContainer.s_Intaker.stop())),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, -4.5), 0, false, true)),
            new WaitCommand(3),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(4.5, 0), 0, false, true)),
            new WaitCommand(3),
            new InstantCommand(() -> m_robotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))

        );
        break;
      case "none":
        break;

    }
  }

}