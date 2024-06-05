
package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * 一個更高級的Swerve控制系統，具有4個按鈕用於指示機器人面向的方向。
 */
public class AbsoluteDriveAdv extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingAdjust;
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;

    /**
     * 用於在全場中心模式下驅動Swerve機器人。vX和vY提供平移輸入，其中x是
     * 向/遠離聯盟牆壁，y是左/右。Heading Adjust用於在被乘以常數後更改當前標題。
     * Look布爾值是獲得機器人面對某個特定方向的快捷方式。
     * 基於
     * https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
     * 的想法
     *
     * @param swerve        Swerve驅動底盤子系統。
     * @param vX            提供x-平移搖桿輸入的DoubleSupplier。
     *                      應該在-1到1的範圍內，已經考慮了死區。正的X是遠離聯盟牆壁。
     * @param vY            提供y-平移搖桿輸入的DoubleSupplier。
     *                      應該在-1到1的範圍內，已經考慮了死區。正的Y是當透過駕駛員站玻璃觀看時，朝向左牆。
     * @param headingAdjust 提供應調整的機器人標題角度的DoubleSupplier組件。
     *                      應該在-1到1的範圍內，已經考慮了死區。
     * @param lookAway      將機器人面向同盟的對立牆壁，與駕駛員面對的方向相同。
     * @param lookTowards   將機器人面向駕駛員。
     * @param lookLeft      將機器人面向左側。
     * @param lookRight     將機器人面向右側。
     */
    public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
            BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
            BooleanSupplier lookRight) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        this.lookAway = lookAway;
        this.lookTowards = lookTowards;
        this.lookLeft = lookLeft;
        this.lookRight = lookRight;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        resetHeading = true;
    }

    // 在計劃的命令被調度時，每次調度器運行時調用。
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;

        // 這些是為了允許45度角的組合而編寫的
        // 面對駕駛員的對立面
        if (lookAway.getAsBoolean()) {
            headingY = -1;
        }
        // 面向右側
        if (lookRight.getAsBoolean()) {
            headingX = 1;
        }
        // 面向左側
        if (lookLeft.getAsBoolean()) {
            headingX = -1;
        }
        // 面向駕駛員
        if (lookTowards.getAsBoolean()) {
            headingY = 1;
        }

        // 防止自動後的移動
        if (resetHeading) {
            if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
                // 獲取當前標題
                Rotation2d currentHeading = swerve.getHeading();

                // 將當前標題設置為所需的標題
                headingX = currentHeading.getSin();
                headingY = currentHeading.getCos();
            }
            // 不要再重置標題
            resetHeading = false;
        }

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

        // 限制速度以防止翻轉
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // 讓機器人移動
        if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
            resetHeading = true;
            swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
        } else {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
        }
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
