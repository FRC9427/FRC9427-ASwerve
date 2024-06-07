
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
/**
 * 這個類是宣告機器人的主要地方。由於基於命令的設計是一種「宣告式」範式，實際上應該在 {@link Robot}
 * 的週期方法中處理的機器人邏輯非常少（除了調度器調用之外）。
 * 相反，機器人的結構（包括子系統、命令和觸發映射）應在此處宣告。
 */
public class RobotContainer {
    // 在這裡定義機器人的子系統和命令...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));

    // 如果需要，替換為 CommandPS4Controller 或 CommandJoystick
    final CommandXboxController driverXbox = new CommandXboxController(0);
    SendableChooser<String> m_Chooser = new SendableChooser<>();

    /**
     * 機器人的容器。包含子系統、操作介面設備和命令。
     */
    public RobotContainer() {
        SmartDashboard.putData("AutonomousPathChoose", m_Chooser);
        // 配置觸發綁定
        configureBindings();

        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox.getHID()::getYButtonPressed,
                driverXbox.getHID()::getAButtonPressed,
                driverXbox.getHID()::getXButtonPressed,
                driverXbox.getHID()::getBButtonPressed);

        // 應用死區並反轉控制，因為搖桿
        // 是右後正，而機器人控制是左前正
        // 左搖桿控制平移
        // 右搖桿控制期望的角度而非角速度
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRightX(),
                () -> driverXbox.getRightY());

        // 應用死區並反轉控制，因為搖桿
        // 是右後正，而機器人控制是左前正
        // 左搖桿控制平移
        // 右搖桿控制機器人的角速度
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRightX());

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    }

    /**
     * 使用此方法定義觸發器與命令的映射。觸發器可以通過
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} 構造函數使用任意謂詞創建，
     * 或通過 {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID} 子類中的命名工廠為
     * {@link CommandXboxController Xbox} /
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} 控制器或
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick} 飛行搖桿創建。
     */
    private void configureBindings() {
        // 當 `exampleCondition` 變為 `true` 時調度 `ExampleCommand`
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        driverXbox.b().whileTrue(
                Commands.deferredProxy(() -> drivebase.driveToPose(
                        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
        // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock,
        // drivebase).repeatedly());
    }

    /**
     * 使用此方法將自動命令傳遞到主 {@link Robot} 類。
     *
     * @return 要在自動模式下運行的命令
     */
    public Command getAutonomousCommand() {
        // 在自動模式下運行的一個示例命令
        return drivebase.getAutonomousCommand("m_Chooser");
    }

    // public void setDriveMode() {
    //     // drivebase.setDefaultCommand();
    // }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
