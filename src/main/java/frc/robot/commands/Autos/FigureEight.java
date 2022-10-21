package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.subsystems.*;

public class FigureEight extends SequentialCommandGroup {

    private final Drivetrain m_drive;

    public FigureEight(Drivetrain drivetrain) {

        m_drive = drivetrain;

        final AutoFromPathPlanner oneBallPath = new AutoFromPathPlanner(drivetrain, "2023swervetest", 2.5, true);

        addCommands(
                new InstantCommand(() -> drivetrain.resetOdometry(oneBallPath.getInitialPose())),
                oneBallPath
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.updateKeepAngle();
        m_drive.stop();
    }
}
