package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

    private final VictorSPX mL = new VictorSPX(Constants.MOTOR_LEFT);
    private final VictorSPX mL2 = new VictorSPX(Constants.MOTOR_LEFT2);
    private final VictorSPX mR = new VictorSPX(Constants.MOTOR_RIGHT);
    private final VictorSPX mR2 = new VictorSPX(Constants.MOTOR_RIGHT2);

    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry area;

    private final Timer m_timer = new Timer();
    private boolean aligning = true;
    private boolean road = true;
    private boolean initSearch = false;
    double y,x;

    @Override
    public void robotInit() {
        initLime();
        initMotors();
    }

    private void initMotors() {
        double deadband = 0.04;
        mL.setInverted(false);
        mL2.setInverted(false);
        mR.setInverted(true);
        mR2.setInverted(true);

        mL.setNeutralMode(NeutralMode.Brake);
        mL2.setNeutralMode(NeutralMode.Brake);
        mR.setNeutralMode(NeutralMode.Brake);
        mR2.setNeutralMode(NeutralMode.Brake);

        mL.configNeutralDeadband(deadband);
        mL2.configNeutralDeadband(deadband);
        mR.configNeutralDeadband(deadband);
        mR2.configNeutralDeadband(deadband);
    }
    public void targetValues(){
        Pose3d targetPose = LimelightHelpers
                .getTargetPose3d_CameraSpace("limelight");
        y = targetPose.getY();
        x = targetPose.getX();

    }
    private void initLime() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        area = table.getEntry("ta");
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return area.getDouble(0.0);
    }

    private void updateLimeDashboard() {
        SmartDashboard.putNumber("Eixo X", getX());
        SmartDashboard.putNumber("Eixo Y", getY());
        SmartDashboard.putNumber("Área", getArea());
    }

    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
        aligning = true;
        road = true;
        initSearch = false;
    }

    @Override
    public void autonomousPeriodic() {
        updateLimeDashboard();
        double taValue = getArea();

        if (taValue == 0.0) {
            searchTag();
        } else if (taValue >= Constants.TA_DIST_MIN && taValue <= Constants.TA_DIST_MAX) {
            aligning = false;
            align(x);
            follower();
        } else if (aligning) {
            searchTag();
        } else {
            stopMotors();
        }
    }

    public void align(double xOffset) {
        double alignSpd = 0.0;
        if (Math.abs(xOffset) > Constants.ALIGN_KP) {
            if (xOffset > Constants.ADJUST_ALIGN) {
                alignSpd = -Constants.ALIGN_SPEED;
            } else if (xOffset < -Constants.ADJUST_ALIGN) {
                alignSpd = Constants.ALIGN_SPEED;
            }
        }

        if (Math.abs(xOffset) <= Constants.ALIGN_KP) {
            alignSpd = 0.0;
        }
        setMotors(alignSpd, -alignSpd);
    }

    public void follower() {
        double taValue = getArea();
        if (taValue < Constants.TA_DIST_MAX && taValue >= Constants.TA_DIST_MIN) {
            moveForward();
        } else if (taValue >= Constants.TA_DIST_MAX) {
            moveBackward();
        } else {
            stopMotors();
        }
    }

    public void searchTag() {
        double goTime = m_timer.get();
        if (!initSearch) {
            if (goTime <= 1.0) {
                moveRight();
            } else if (goTime <= 2.0) {
                moveLeft();
            } else {
                initSearch = true;
                m_timer.reset();
            }
        } else {
            if (road) {
                moveRight();
            } else {
                moveLeft();
            }

            if (goTime > 6.0) {
                road = !road;
                m_timer.reset();
            }
        }
    }

    public void autoAlign() {
        double xOffset = getX();
        align(xOffset);
        follower();
    }
    public void setMotors(double lSpd, double rSpd) {
        mL.set(ControlMode.PercentOutput, lSpd);
        mL2.set(ControlMode.PercentOutput, lSpd);
        mR.set(ControlMode.PercentOutput, rSpd);
        mR2.set(ControlMode.PercentOutput, rSpd);
    }

    public void moveForward() {
        setMotors(Constants.SPD_ADJUST, Constants.SPD_ADJUST);
    }

    public void moveBackward() {
        setMotors(-Constants.SPD_ADJUST, -Constants.SPD_ADJUST);
    }

    public void moveLeft() {
        setMotors(-Constants.SPD_ADJUST * Constants.K_VALUE, Constants.SPD_ADJUST * Constants.K_VALUE);
    }

    public void moveRight() {
        setMotors(Constants.SPD_ADJUST * Constants.K_VALUE, -Constants.SPD_ADJUST * Constants.K_VALUE);
    }

    public void stopMotors() {
        setMotors(0, 0);
    }
}