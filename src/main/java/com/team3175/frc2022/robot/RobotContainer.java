// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3175.frc2022.robot;

import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.autos.automodes.*;
import com.team3175.frc2022.robot.commands.*;
import com.team3175.frc2022.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

@SuppressWarnings("unused")
public class RobotContainer {

  private static SendableChooser<Command> autoChooser;

  /* Controllers */
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_PORT);
  private final XboxController m_opController = new XboxController(Constants.OPERATOR_PORT);

  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_feedShooter = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  /* Operator Buttons */
  private final JoystickButton m_intakeCargo = new JoystickButton(m_opController, XboxController.Button.kY.value);
  private final JoystickButton m_outtakeCargo = new JoystickButton(m_opController, XboxController.Button.kX.value);
  private final JoystickButton m_shootCargo = new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value);
  private final POVButton m_climbUp = new POVButton(m_opController, 0);
  private final POVButton m_climbDown = new POVButton(m_opController, 180);
  private final JoystickButton m_lockClimber = new JoystickButton(m_opController, XboxController.Button.kStart.value);
  private final JoystickButton m_unlockClimber = new JoystickButton(m_opController, XboxController.Button.kBack.value);
  private final JoystickButton m_passiveHooksUp = new JoystickButton(m_opController, XboxController.Button.kRightStick.value);
  private final JoystickButton m_passiveHooksDown = new JoystickButton(m_opController, XboxController.Button.kB.value);

  /* Subsystems */
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Actuators m_actuator = new Actuators();
  private final Climber m_climber = new Climber();
  private final Diagnostics m_diagnostics;

  /* Autos */

  //Basic 1 ball
  private final Command m_oneBall = new OneBall(m_shooter, m_feeder, m_swerveDrivetrain);

  //Three balls
  private final Command m_threeBallBCBlue = new ThreeBallBCBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_threeBallBCRed = new ThreeBallBCRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //Four ball hangar side
  private final Command m_fourBallADEBlue = new FourBallADEBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_fourBallADERed = new FourBallADERed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //Four ball terminal side
  private final Command m_fourBallBCDBlue = new FourBallBCDBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_fourBallBCDRed = new FourBallBCDRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //Three ball gamer
  private final Command m_threeBallDEBlue = new ThreeBallDEBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_threeBallDERed = new ThreeBallDERed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //Two ball defensive hangar
  private final Command m_twoBallADefenseHangarBlue = new TwoBallADefenseHangarBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_twoBallADefenseHangarRed = new TwoBallADefenseHangarRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //Two ball defensive hub
  private final Command m_twoBallADefenseHubBlue = new TwoBallADefenseHubBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_twoBallADefenseHubRed = new TwoBallADefenseHubRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //One ball defensive hangar
  private final Command m_oneBallDefenseHangarBlue = new OneBallDefenseHangarBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_oneBallDefenseHangarRed = new OneBallDefenseHangarRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);

  //One ball defensive hub
  private final Command m_oneBallDefenseHubBlue = new OneBallDefenseHubBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  private final Command m_oneBallDefenseHubRed = new OneBallDefenseHubRed(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  
  //Corn Dog
  //private final Command m_cornDogBlue = new CornDogBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);
  //private final Command m_cornDogRed = new CornDogBlue(m_shooter, m_feeder, m_intake, m_actuator, m_swerveDrivetrain);


  public RobotContainer(){

    /* Set Drive as default command*/
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));

    /* Initialize diagnostics subystem */
    m_diagnostics = new Diagnostics(m_swerveDrivetrain, m_climber, m_intake, m_feeder, m_shooter, m_actuator);

    /* Add all autons to AutoChooser */
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("One Ball Auto", m_oneBall);
    autoChooser.addOption("Three Ball BC Red (regular 3 ball)", m_threeBallBCRed);
    autoChooser.addOption("Three Ball BC Blue (regular 3 ball)", m_threeBallBCBlue);
    autoChooser.addOption("Four Ball ADE (wack) Red", m_fourBallADERed);
    autoChooser.addOption("Four Ball ADE (wack) Blue", m_fourBallADEBlue);
    autoChooser.addOption("Four Ball BCD (terminal) Red", m_fourBallBCDRed);
    autoChooser.addOption("Four Ball BCD (terminal) Blue", m_fourBallBCDBlue);
    autoChooser.addOption("Three Ball DE (gamer) Red", m_threeBallDERed);
    autoChooser.addOption("Three Ball DE (gamer) Blue", m_threeBallDEBlue);
    autoChooser.addOption("Two Ball A Defense Hangar Blue", m_twoBallADefenseHangarBlue);
    autoChooser.addOption("Two Ball A Defense Hangar Red", m_twoBallADefenseHangarRed);
    autoChooser.addOption("Two Ball A Defense Hub Blue", m_twoBallADefenseHubBlue);
    autoChooser.addOption("Two Ball A Defense Hub Red", m_twoBallADefenseHubRed);
    autoChooser.addOption("One Ball Defense Hangar Blue", m_oneBallDefenseHangarBlue);
    autoChooser.addOption("One Ball Defense Hangar Red", m_oneBallDefenseHangarRed);
    autoChooser.addOption("One Ball Defense Hub Blue", m_oneBallDefenseHubBlue);
    autoChooser.addOption("One Ball Defense Hub Red", m_oneBallDefenseHubRed);
    //autoChooser.addOption("Corn Dog Blue", m_cornDogBlue);
    //autoChooser.addOption("Corn Dog Red", m_cornDogRed);
    SmartDashboard.putData("Auto mode", autoChooser);
    
    /* Configure the button bindings */
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */

    //Zero Gyro -> X Button
    m_zeroGyro.whenPressed(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));

    //Feeder -> A Button
    m_feedShooter.whenPressed(new InstantCommand(() -> m_feeder.feederRunVelocity(Constants.TARGET_FEEDER_RPM)))
                  .whenReleased(new InstantCommand(() -> m_feeder.feederRunPercentOutput(0)));
                  
    /* Operator Buttons */ 

    //Intake -> Y Button
    m_intakeCargo.whenPressed(new IntakeCargo(m_intake, Constants.INTAKE_SPEED, m_opController))
                 .whenReleased(new IntakeCargo(m_intake, 0, m_opController));

    //Outtake -> X Button
    m_outtakeCargo.whenPressed(new IntakeCargo(m_intake, Constants.OUTTAKE_SPEED, m_opController))
                  .whenReleased(new IntakeCargo(m_intake, 0, m_opController));

    //Outtake Actuation -> X Button
    m_outtakeCargo.whenPressed(new ActuateIntake(m_actuator))
                  .whenReleased(new ActuateBack(m_actuator));

    //Shoot -> Left Bumper
    m_shootCargo.whenPressed(new ShootCargo(m_shooter, Constants.SHOOTER_TARGET_RPM, m_driverController, m_opController))
                .whenReleased(new StopShooter(m_shooter, m_driverController, m_opController));

    //Intake Actuation -> Linked to intake (Y Button)
    m_intakeCargo.whenPressed(new ActuateIntake(m_actuator))
                 .whenReleased(new ActuateBack(m_actuator));

    //Climber Up -> Dpad 0
    m_climbUp.whenPressed(new ClimbUp(m_climber, Conversions.climberInchesToEncoders(Constants.CLIMBER_UP_DISTANCE), Constants.CLIMBER_SPEED))
             .whenReleased(new InstantCommand(() -> m_climber.overrideStop()));

    //Climber Down -> Dpad 180
    m_climbDown.whenHeld(new OverrideClimbDown(m_climber, Constants.CLIMBER_SPEED))
                       .whenReleased(new InstantCommand(() -> m_climber.overrideStop()));

    //Climber Lock -> Start Button
    m_lockClimber.whenPressed(new InstantCommand(() -> m_climber.lockPneumatics()));

    //Climber Unlock -> Back Button
    m_unlockClimber.whenPressed(new InstantCommand(() -> m_climber.unlockPneumatics()));

    //Passive Hooks Release -> Right Joystick Button
    m_passiveHooksUp.whenPressed(new SetHookState(m_climber, "up"));

    //Passive Hooks Lock -> B Button
    m_passiveHooksDown.whenPressed(new SetHookState(m_climber, "down"));


  }

  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
  
}