
package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.AutonConstants;
import java.io.File;
import java.util.function.DoubleSupplier;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    /**
     * Swerve 驅動對象。
     */
    private final SwerveDrive swerveDrive;
    /**
     * 機器人最大速度（米/秒），用於限制加速度。
     */
    public double maximumSpeed = 6.8;

    /**
     * 使用提供的目錄初始化 {@link SwerveDrive}。
     *
     * @param directory Swerve 驅動配置文件的目錄。
     */
    public SwerveSubsystem(File directory) {

        // 角度轉換因子為 360 / (齒輪比 * 編碼器解析度),如果使用maxswerve則一定是360
        double angleConversionFactor = 360;
        // 馬達轉換因子是 (PI * 輪子直徑（米）) / (齒輪比 * 編碼器解析度)。
        double driveConversionFactor = 0.058033784291767816;
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");

        // 在創建 SwerveDrive 之前配置遙測，以避免創建不必要的對象。
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;//測試完成即可改成low
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
            // 如果不想通過 JSON 文件提供轉換因子，可以使用替代方法。
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // 方向校正應僅在通過角度控制機器人時使用
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // 在模擬中禁用餘弦補償，因為它會導致實際中看不到的差異。
        setupPathPlanner();
        // 250hz的里程計
        swerveDrive.setOdometryPeriod(0.004);
    }

    /**
     * SwreveSubsystem 建構式
     *
     * @param driveCfg      SwerveDrive 的配置。
     * @param controllerCfg Swerve 控制器。
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    /**
     * 為 PathPlanner 設置 AutoBuilder。
     */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        AutonConstants.TRANSLATION_PID,
                        AutonConstants.ANGLE_PID,

                        4.5,
                        0.4,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    // /**
    //  * 使機器人對準 PhotonVision 返回的目標。
    //  *
    //  * @param camera 與之通信的 {@link PhotonCamera}。
    //  * @return 將運行對齊的 {@link Command}。
    //  */
    // public Command aimAtTarget(PhotonCamera camera) {
    //     return run(() -> {
    //         PhotonPipelineResult result = camera.getLatestResult();
    //         if (result.hasTargets()) {
    //             drive(getTargetSpeeds(0,
    //                     0,
    //                     Rotation2d.fromDegrees(result.getBestTarget()
    //                             .getYaw()))); // 隨便算得不太確定
    //         }
    //     });
    // }

    /**
     * 獲取具有事件的路徑跟隨器。
     *
     * @param pathName PathPlanner 路徑名稱。
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} 路徑命令。
     */
    public Command getAutonomousCommand(String pathName) {
        // 使用 AutoBuilder 創建路徑跟隨命令。這還將觸發事件標記。
        return new PathPlannerAuto(pathName);
    }

    /**
     * 使用 PathPlanner 路徑規劃前往場地上的一個點。
     *
     * @param pose 目標的 {@link Pose2d}，即要前往的位置。
     * @return 路徑尋找指令
     */
    public Command driveToPose(Pose2d pose) {
        // 創建在路徑尋找時使用的約束條件
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumVelocity(), 4.0,
                swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

        // 由於已經配置了 AutoBuilder，我們可以使用它來構建路徑尋找指令
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // 目標終端速度，以米/秒為單位
                0.0 // 旋轉延遲距離，以米為單位。這是機器人應該在嘗試旋轉之前行駛的距離。
        );
    }

    /**
     * 使用平移值和方位作為設定點來駕駛機器人的指令。
     *
     * @param translationX X 方向的平移量。立方以使控制更平滑。
     * @param translationY Y 方向的平移量。立方以使控制更平滑。
     * @param headingX     用於計算搖桿角度的 X 方位值。
     * @param headingY     用於計算搖桿角度的 Y 方位值。
     * @return 駕駛指令。
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // 通常你會希望在這種控制中使用方位校正。
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3); // 平滑控制
            double yInput = Math.pow(translationY.getAsDouble(), 3); // 平滑控制
            // 使機器人移動
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * 使用平移值和方位作為設定點來駕駛機器人的指令。
     *
     * @param translationX X 方向的平移量。
     * @param translationY Y 方向的平移量。
     * @param rotation     旋轉值，範圍在 [-1, 1] 之間，將轉換為弧度。
     * @return 駕駛指令。
     */
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // swerveDrive.setHeadingCorrection(true); // 通常你會希望在這種控制中使用方位校正。
        return run(() -> {
            // 使機器人移動
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * 使用 SysId 來表徵機器人驅動馬達的指令
     *
     * @return SysId 驅動指令
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12),
                3.0, 5.0, 3.0);
    }

    /**
     * 使用 SysId 來表徵機器人的角度馬達的指令
     *
     * @return SysId 角度指令
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * 使用平移值和方位角速度作為設定點來駕駛機器人的指令。
     *
     * @param translationX     X 方向的平移量。取三次方以獲得更平滑的控制。
     * @param translationY     Y 方向的平移量。取三次方以獲得更平滑的控制。
     * @param angularRotationX 機器人的角速度。取三次方以獲得更平滑的控制。
     * @return 駕駛指令。
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // 使機器人移動
            swerveDrive.drive(
                    new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                            Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false);
        });
    }

    /**
     * 控制驅動基座的主要方法。接受一個 {@link Translation2d} 和旋轉速率，並相應地計算和命令模組狀態。
     * 可以使用開環或閉環速度控制來控制車輪速度。也有場地相對和機器人相對模式，這會影響平移向量的使用方式。
     *
     * @param translation   {@link Translation2d}，這是機器人的命令線速度，以米每秒為單位。
     *                      在機器人相對模式下，正的 x 方向朝向船頭（前方），正的 y 方向朝向左舷（左側）。
     *                      在場地相對模式下，正的 x 方向遠離聯盟牆（場地北），正的 y 方向朝向駕駛員站玻璃的左牆（場地西）。
     * @param rotation      機器人的角速度，以弧度每秒為單位。逆時針為正。不受場地/機器人相對模式影響。
     * @param fieldRelative 驅動模式。場地相對為 true，機器人相對為 false。
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // 關閉開環模式，因為它不應該經常使用。
    }

    /**
     * 根據底盤的場地相對速度來駕駛機器人。
     *
     * @param velocity 根據場地的速度。
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * 根據底盤的機器人相對速度來駕駛。
     *
     * @param velocity 機器人相對的 {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void periodic() {
        // swerveDrive.pushOffsetsToControllers();
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * 獲取SwerveDrive的運動學對象。
     *
     * @return SwerveDrive的 {@link SwerveDriveKinematics}。
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * 重置里程計到給定的位置。調用此方法時不需要重置陀螺儀角度和模組位置。
     * 然而，如果重置了陀螺儀角度或模組位置，必須調用此方法以保持里程計的正常工作。
     *
     * @param initialHolonomicPose 要設定的里程計位置
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * 獲取機器人的當前位置（包括位置和旋轉），由里程計報告。
     *
     * @return 機器人的位置
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * 使用閉環速度控制設定底盤速度。
     *
     * @param chassisSpeeds 要設定的底盤速度。
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * 將軌跡發佈到場地。
     *
     * @param trajectory 要發佈的軌跡。
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * 將陀螺儀角度重置為零，並將里程計重置到相同的位置，但面向 0 度。
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * 設定驅動馬達為制動/滑行模式。
     *
     * @param brake 設為 true 表示設置馬達為制動模式，false 表示滑行模式。
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * 獲取機器人的當前偏航角度，由底盤中的萬向姿態估計器報告。
     * 注意，這不是原始的陀螺儀讀數，這可能已經通過調用 resetOdometry() 進行修正。
     *
     * @return 偏航角度
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * 根據兩個搖桿的控制輸入獲取底盤速度。一個用於方向上的速度，另一個用於機器人的角度。
     *
     * @param xInput   控制機器人在 X 方向移動的搖桿輸入。
     * @param yInput   控制機器人在 Y 方向移動的搖桿輸入。
     * @param headingX 控制機器人角度的 X 搖桿。
     * @param headingY 控制機器人角度的 Y 搖桿。
     * @return 可發送到萬向驅動的 {@link ChassisSpeeds}。
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * 根據 1 個搖桿和一個角度的控制器輸入獲取底盤速度。
     * 以偏移 90 度控制機器人。
     *
     * @param xInput 控制機器人在 X 方向移動的搖桿輸入。
     * @param yInput 控制機器人在 Y 方向移動的搖桿輸入。
     * @param angle  以 {@link Rotation2d} 表示的角度。
     * @return 可發送到Swerve驅動的 {@link ChassisSpeeds}。
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * 獲取機器人當前的場地相對速度（x, y 和 omega）。
     *
     * @return 表示當前場地相對速度的 ChassisSpeeds 對象。
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * 獲取機器人當前的速度（x, y 和 omega）。
     *
     * @return 表示當前速度的 {@link ChassisSpeeds} 對象。
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * 獲取萬向驅動中的 {@link SwerveController}。
     *
     * @return 來自 {@link SwerveDrive} 的 {@link SwerveController}。
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * 獲取 {@link SwerveDriveConfiguration} 對象。
     *
     * @return 當前驅動的 {@link SwerveDriveConfiguration}。
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * X pose
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * 獲取機器人的當前俯仰角度，由 imu 報告。
     *
     * @return 以 {@link Rotation2d} 角度表示的俯仰角。
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    /**
     * 添加一個假的視覺讀數以進行測試。
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }
}