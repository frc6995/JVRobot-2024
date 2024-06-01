package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.subsystems.drive.DrivebaseS;
import frc.robot.subsystems.drive.Pathing;
import frc.robot.subsystems.intake.IntakeRollerS;
import frc.robot.subsystems.vision.BlobDetectionCamera;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.InputAxis;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.TimingTracer;
import frc.robot.util.sparkmax.SparkDevice;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.Set;
import java.util.function.Consumer;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer implements Logged {

  /** Establishes the controls and subsystems of the robot */
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_testingController = new CommandXboxController(5);
  private final CommandXboxController m_keypad = new CommandXboxController(2);
  private final DriverDisplay m_driverDisplay = new DriverDisplay();
  private final DrivebaseS m_drivebaseS;
  private final IntakeRollerS m_intakeRollerS;

  @Log.NT
  private final Mechanism2d MECH_VISUALIZER = RobotVisualizer.MECH_VISUALIZER;
  private final BlobDetectionCamera m_noteCamera;
  private final LightStripS m_lightStripS;
  @Log.NT
  private double loopTime = 0;
  private LinearFilter loopTimeAverage = LinearFilter.movingAverage(1);
  @Log.NT
  private final Field2d m_field = new Field2d();
  @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
  private final Field2d m_driverField = new Field2d();

  private final CommandGroups m_autos;

  private InputAxis m_fwdXAxis = new InputAxis("Forward", m_driverController::getLeftY)
      .withDeadband(0.1)
      .withInvert(true)
      .withSlewRate(3)
      .withSquaring(true);
  private InputAxis m_fwdYAxis = new InputAxis("Strafe", m_driverController::getLeftX)
      .withDeadband(0.1)
      .withInvert(true)
      .withSlewRate(3)
      .withSquaring(true);
  private InputAxis m_rotAxis = new InputAxis("Rotate", m_driverController::getRightX)
      .withDeadband(0.2)
      .withInvert(true)
      .withSlewRate(1.33, -6);
  @Log
  SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

  private boolean m_setupDone = false;

  @Log.NT
  private double getFwdAxis() {
    return m_fwdXAxis.getAsDouble();
  }

  @Log.NT
  private double getSideAxis() {
    return m_fwdYAxis.getAsDouble();
  }

  @Log.NT
  private double getRotAxis() {
    return m_rotAxis.getAsDouble();
  }

  public RobotContainer(Consumer<Runnable> addPeriodic) {
    if (true || RobotBase.isSimulation()) {
      PhotonCamera.setVersionCheckEnabled(false);
    }
    m_lightStripS = LightStripS.getInstance();
    
    m_drivebaseS = new DrivebaseS(
        addPeriodic,
        (name, poses) -> m_field.getObject(name).setPoses(poses));
    m_intakeRollerS = new IntakeRollerS();
    m_noteCamera = new BlobDetectionCamera(addPeriodic, m_field.getObject("note"));

    m_autos = new CommandGroups(
        m_drivebaseS,
        m_intakeRollerS,
        m_noteCamera,
        m_lightStripS);
    configureDriverDisplay();
    configureButtonBindings();
    addAutoRoutines();

    RobotVisualizer.setupVisualizer();
    RobotVisualizer.addIntake(m_intakeRollerS.INTAKE_ROLLER);

    SmartDashboard.putData(m_autoSelector);
    Monologue.setupMonologue(this, "Robot", false, true);
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
    SparkDevice.burnFlashInSync();
    Commands.sequence(waitSeconds(4), runOnce(() -> m_setupDone = true))
        .ignoringDisable(true)
        .schedule();
    DriverStation.reportWarning("Setup Done", false);
    // m_shooterPivotS.setDefaultCommand(m_shooterPivotS.rotateWithVelocity(
    // this::pivotAngle,
    // ()-> Interpolation.dThetadX(distanceToSpeaker()) *
    // -Pathing.velocityTorwardsSpeaker(
    // m_drivebaseS.getPose(), m_drivebaseS.getFieldRelativeLinearSpeedsMPS(),
    // speaker())
    // )
    // );
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("pathplanner").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> m_field.getObject("ppTarget").setPose(pose));
  }

  public void configureDriverDisplay() {
    m_driverDisplay.setSeeNoteSupplier(m_noteCamera.hasTarget);
  }

  public Translation2d speaker() {
    return m_autos.speaker();
  }

  @Log
  public double directionToSpeaker() {
    return m_autos.directionToSpeaker();
  }

  @Log
  public double distanceToSpeaker() {
    return m_autos.distanceToSpeaker();
  }

  public Command faceSpeaker() {
    return m_drivebaseS.manualFieldHeadingDriveC(m_fwdXAxis, m_fwdYAxis,
        this::directionToSpeaker,
        () -> Pathing.aimingFFVelocity(
            m_drivebaseS.getPose(),
            m_drivebaseS.getFieldRelativeLinearSpeedsMPS(), NomadMathUtil.mirrorTranslation(
                Constants.Poses.SPEAKER,
                AllianceWrapper.getAlliance())));
  }

  public Command faceNote() {
    return m_autos.faceNoteC(m_fwdXAxis, m_fwdYAxis,
                rumble -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumble));
  }
  public void configureButtonBindings() {
    m_drivebaseS.setDefaultCommand(m_drivebaseS.manualDriveC(m_fwdXAxis, m_fwdYAxis, m_rotAxis));
    m_driverController.a().whileTrue(faceSpeaker());
    m_driverController.b().whileTrue(faceNote());

    m_driverController.rightBumper().whileTrue(m_intakeRollerS.intakeC());
    
    m_driverController.povCenter().negate().whileTrue(driveIntakeRelativePOV());
  }

  public Command driveIntakeRelativePOV() {
    return m_drivebaseS.run(() -> {
      double pov = Units.degreesToRadians(-m_driverController.getHID().getPOV());
      double adjustSpeed = 0.75; // m/s
      m_drivebaseS.drive(
          new ChassisSpeeds(
              Math.cos(pov) * adjustSpeed,
              Math.sin(pov) * adjustSpeed,
              m_rotAxis.getAsDouble() * DriveConstants.MAX_TURN_SPEED));
    });
  }

  public void addAutoRoutines() {
    m_autoSelector.setDefaultOption("Do Nothing", none());
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.getSelected();
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (m_setupDone) {
        LightStripS.getInstance().requestState(States.SetupDone);
      } else {
        LightStripS.getInstance().requestState(States.Disabled);
      }
    }
    TimingTracer.update();
    loopTime = loopTimeAverage.calculate(TimingTracer.getLoopTime());
    // /* Trace the loop duration and plot to shuffleboard */
    LightStripS.getInstance().periodic();
    updateFields();
    var beforeLog = Timer.getFPGATimestamp();
    Monologue.updateAll();
    var afterLog = Timer.getFPGATimestamp();
    log("mlUpdate", (afterLog - beforeLog));
    m_driverDisplay.update();
  }

  public void updateFields() {
    m_drivebaseS.drawRobotOnField(m_field);
    m_driverField.getRobotObject().setPose(m_drivebaseS.getPose());
    m_field.getObject("note").setPoses(m_noteCamera.getTargets(m_drivebaseS::getOldPose));
    m_field.getObject("driveTarget").setPose(m_drivebaseS.getTargetPose());
  }

  public void onEnabled() {
    m_drivebaseS.resetRelativeRotationEncoders();
  }

  public void onDisabled() {

  }

}
