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

public class OneBallDefenseHangarRed extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private SwerveDrivetrain m_drivetrain;
    private Intake m_intake;
    private Actuators m_actuators;
    private PathPlannerTrajectory m_driveBack;
    private PathPlannerTrajectory m_driveToHangar;
    private Pose2d m_initialPose;

    public OneBallDefenseHangarRed(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;
        m_initialPose = new Pose2d(7.11, 4.57, Rotation2d.fromDegrees(-20.56));

        m_driveBack = PathPlanner.loadPath("1Ball-Defense-1-Hangar-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_driveToHangar = PathPlanner.loadPath("1Ball-Defense-2-Hangar-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand m_driveCommand = 
            new PPSwerveControllerCommand(
            m_driveBack, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_driveToHangarCommand = 
            new PPSwerveControllerCommand(
            m_driveToHangar, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUp = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeed = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(m_initialPose)),
                    new WaitCommand(5.0),
                    m_spinUp,
                    m_shootAndFeed,
                    new ParallelCommandGroup(m_driveCommand, new SetIntakeState(m_intake, m_actuators, "deploy", 0.1)),
                    new ParallelCommandGroup(m_driveToHangarCommand, new SetIntakeState(m_intake, m_actuators, "retract", 0.0)),
                    new StopSwerve(m_drivetrain),
                    new SetIntakeState(m_intake, m_actuators, "deploy reverse", 0.7),
                    new InstantCommand(() -> m_drivetrain.setGyro(117.35)));

    }

    
}
