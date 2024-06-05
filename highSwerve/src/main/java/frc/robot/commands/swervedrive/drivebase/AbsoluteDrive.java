
package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * 一個使用示例子系統的示例命令。
 */
public class AbsoluteDrive extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private boolean initRotation = false;

    /**
     * 用於在全場中心模式下驅動Swerve機器人。vX和vY提供平移輸入，其中x是
     * 向/遠離聯盟牆壁，y是左/右。headingHorzontal和headingVertical是機器人角度的笛卡爾坐標，
     * 它們將被轉換為極角，機器人將旋轉到該角度。
     *
     * @param swerve            The swerve drivebase subsystem.
     * @param vX                提供x-平移搖桿輸入的DoubleSupplier。
     *                          應該在-1到1的範圍內，已經考慮了死區。正的X是遠離聯盟牆壁。
     * @param vY                提供y-平移搖桿輸入的DoubleSupplier。
     *                          應該在-1到1的範圍內，已經考慮了死區。正的Y是當透過駕駛員站玻璃觀看時，朝向左牆。
     * @param headingHorizontal 提供機器人標題角度的水平分量的DoubleSupplier。
     *                          在機器人坐標系中，這與vY沿著同一軸。應該在-1到1的範圍內，不帶死區。
     *                          正的是當透過駕駛員站玻璃觀看時，向左牆。
     * @param headingVertical   提供機器人標題角度的垂直分量的DoubleSupplier。
     *                          在機器人坐標系中，這與vX沿著同一軸。應該在-1到1的範圍內，不帶死區。
     *                          正的是遠離聯盟牆壁。
     */
    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
            DoubleSupplier headingVertical) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initRotation = true;
    }

    // 在計劃的命令被調度時，每次調度器運行時調用。
    @Override
    public void execute() {

        // 根據2個搖桿模塊獲取期望的底盤速度。
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                headingHorizontal.getAsDouble(),
                headingVertical.getAsDouble());

        // 防止自動後的移動
        if (initRotation) {
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
                // 獲取當前標題
                Rotation2d firstLoopHeading = swerve.getHeading();

                // 將當前標題設置為所需的標題
                desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
            }
            // 不要再初始化旋轉
            initRotation = false;
        }

        // 限制速度以防止翻轉
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // 讓機器人移動
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

    }

    // 在命令結束或被中斷時調用一次。
    @Override
    public void end(boolean interrupted) {
    }

    // 當命令應該結束時返回true。
    @Override
    public boolean isFinished() {
        return false;
    }

}
