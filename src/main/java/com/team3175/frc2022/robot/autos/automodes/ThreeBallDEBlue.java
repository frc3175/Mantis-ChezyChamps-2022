package com.team3175.frc2022.robot.autos.automodes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.autos.autocommands.AutonShootAndFeed;
import com.team3175.frc2022.robot.autos.autocommands.AutonSpinUp;
import com.team3175.frc2022.robot.commands.SetIntakeState;
import com.team3175.frc2022.robot.commands.StopSwerve;
import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Intake;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeBallDEBlue extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_trajectory;
    private PathPlannerTrajectory m_trajectory2;
    private PathPlannerTrajectory m_trajectory3;
    private PathPlannerTrajectory m_trajectory4;

    public ThreeBallDEBlue(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;

        m_trajectory = PathPlanner.loadPath("3BallDE-1-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_trajectory2 = PathPlanner.loadPath("3BallDE-2-Blue", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_trajectory3 = PathPlanner.loadPath("3BallDE-3-Blue", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_trajectory4 = PathPlanner.loadPath("3BallDE-4-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand m_trajectoryCommand = 
            new PPSwerveControllerCommand(
            m_trajectory, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_trajectoryCommand2 = 
            new PPSwerveControllerCommand(
            m_trajectory2, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_trajectoryCommand3 = 
            new PPSwerveControllerCommand(
            m_trajectory3, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_trajectoryCommand4 = 
            new PPSwerveControllerCommand(
            m_trajectory4, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUp1 = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeed1 = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);
        AutonShootAndFeed m_shootAndFeed2 = new AutonShootAndFeed(m_shooter, m_feeder, (Constants.FEEDER_TICKS * 2), Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        SetIntakeState m_intakeDeploy = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);

        SetIntakeState m_intakeRetract = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(7.11, 4.57, Rotation2d.fromDegrees(-20.56)))),
                    new InstantCommand(() -> m_feeder.resetEncoders()),
                    new WaitCommand(3.0),
                    m_spinUp1,
                    m_shootAndFeed1,
                    new ParallelCommandGroup(m_trajectoryCommand),
                    new ParallelCommandGroup(m_trajectoryCommand2, m_intakeDeploy),
                    new ParallelCommandGroup(m_trajectoryCommand3),
                    new StopSwerve(m_drivetrain),
                    new WaitCommand(1.5),
                    new ParallelCommandGroup(m_trajectoryCommand4, m_intakeRetract, new InstantCommand(() -> m_shooter.shoot(Constants.SHOOTER_TARGET_RPM))),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeed2,
                    new InstantCommand(() -> m_drivetrain.setGyro(339.44))
                    );

    }

    
}
