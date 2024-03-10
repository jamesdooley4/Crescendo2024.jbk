package frc.team2412.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.util.MACAddress;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
	private static Robot instance = null;

	public enum RobotType {
		COMPETITION,
		PRACTICE,
		CRANE,
		BONK;
	}

	public static Robot getInstance() {
		if (instance == null) instance = new Robot();
		return instance;
	}

	private static final boolean debugMode = true;

	private final RobotType robotType;
	private final SwerveDrive swerveDrive;

	public SendableChooser<Command> autoChooser;

	protected Robot(RobotType type) {
		// non public for singleton. Protected so test class can subclass
		instance = this;
		robotType = type;

		File swerveJsonDirectory;

		switch (type) {
			case PRACTICE:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "practiceswerve");
				System.out.println("Running practice swerve");
				break;
			case CRANE:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "craneswerve");
				System.out.println("Running crane swerve");
				break;
			case BONK:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "bonkswerve");
				System.out.println("Running bonk swerve");
				break;
			case COMPETITION:
			default:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
				System.out.println("Running competition swerve");
		}

		final double MAX_SPEED =
		type == RobotType.BONK
				? 3.0
				: Robot.getInstance().getRobotType() == RobotType.PRACTICE
						? 5.0
						: Robot.getInstance().getRobotType() == RobotType.CRANE ? 3.0 : 1.0;


		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED);
		} catch (Exception e) {
			System.out.println(e);
			throw new RuntimeException();
		}

		// set drive motors to coast intially, this will be changed to brake on enable
		swerveDrive.setMotorIdleMode(false);
		// swerve drive heading will slowly drift over time as you translate. this method enables an
		// active correction using pid. disabled until testing can be done
		// TODO: this still needs to be improved
		swerveDrive.setHeadingCorrection(true, 2.0, 0.5);
		swerveDrive.chassisVelocityCorrection = true;

		// LOW verbosity only sends field position, HIGH sends full drive data, MACHINE sends data
		// viewable by AdvantageScope
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
	}

	protected Robot() {
		this(getTypeFromAddress());
	}

	public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x00, 0x00, 0x00);
	public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x33, 0x9d, 0xD1);
	public static final MACAddress BONK_ADDRESS = MACAddress.of(0x33, 0x9D, 0xE7);
	public static final MACAddress CRANE_ADDRESS = MACAddress.of(0x22, 0xB0, 0x92);

	private static RobotType getTypeFromAddress() {
		return RobotType.PRACTICE;
		// if (PRACTICE_ADDRESS.exists()) return RobotType.PRACTICE;
		// if (CRANE_ADDRESS.exists()) return RobotType.CRANE;
		// if (BONK_ADDRESS.exists()) return RobotType.BONK;
		// if (!COMPETITION_ADDRESS.exists())
		// 	DriverStation.reportWarning(
		// 			"Code running on unknown MAC Address! Running competition code anyways", false);
		// return RobotType.COMPETITION;
	}

	@Override
	public void robotInit() {
		Pose2d pose = new Pose2d();
		swerveDrive.resetOdometry(pose);

		Rotation2d rotation = new Rotation2d();
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i=0; i<1000; i++) {
			positions[0].distanceMeters = i * 0.01;
			positions[1].distanceMeters = i * 0.01;
			positions[2].distanceMeters = i * 0.01;
			positions[3].distanceMeters = i * 0.01;
			swerveDrive.updateOdometryFromLogs(i * 0.01, rotation, null);

			System.out.println("Pose: " + swerveDrive.odometryOnlyPoseEstimator.getEstimatedPosition());
		}
	}

	@Override
	public void testInit() {}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void teleopInit() {}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledExit() {}

	public RobotType getRobotType() {
		return robotType;
	}

	public boolean isCompetition() {
		return getRobotType() == RobotType.COMPETITION;
	}

	public static boolean isDebugMode() {
		return debugMode;
	}
}
