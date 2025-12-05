package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configs.Config;

public class Turret {
    private Config config;
    private DcMotorEx turretRotater;
    private Servo leftHoodServo;
    private Servo rightHoodServo;
    private DcMotorEx leftFlywheelMotor;
    private DcMotorEx rightFlywheelMotor;
    private Limelight3A limelight;
    public Turret(HardwareMap hardwareMap, int alliance){
        //initalize turret servos and motor
        config = new Config();
        leftHoodServo = hardwareMap.get(Servo.class, config.leftHoodServo);
        rightHoodServo = hardwareMap.get(Servo.class, config.rightHoodServo);
        //set motor runMode
        turretRotater = hardwareMap.get(DcMotorEx.class, config.turretRotationName);
        turretRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //initalize left motor
        leftFlywheelMotor = hardwareMap.get(DcMotorEx.class, config.newRobotLeftFlywheel);
        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //initalize right motor
        rightFlywheelMotor = hardwareMap.get(DcMotorEx.class, config.newRobotRightFlywheel);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //initalize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.resetDeviceConfigurationForOpMode();
        limelight.start();
        if(alliance == config.RedAlliance){
            limelight.pipelineSwitch(0);
        }
        else if(alliance == config.BlueAlliance){
            limelight.pipelineSwitch(1);
        }

    }
    public double setTurretPositionDegrees(double degrees){
        //TODO get ticks per rotation of turret
        //make it so turret an never exceed 180 degrees or go below -180 degrees
        //make it so 0 degrees is straightforward as to avoid confusion with the current aiming system
        return 0.0;
    }

    /**
     * sets the hood to a specific launch angle
     * @param givenLaunchAngle is the desired launch angle
     */
    public void setHoodLaunchAngle(double givenLaunchAngle){
        double servoRange = config.hoodMaximumLaunchAngle - config.hoodMinimumLaunchAngle;
        double givenPosition = givenLaunchAngle/config.hoodMaximumLaunchAngle;
        rightHoodServo.setPosition(givenPosition);
        leftHoodServo.setPosition(1-givenPosition);
    }
    public void setFlywheelsToRPM(int rpm){
        int ticksPerRotation = config.ticksPerRevFlywheel;
        int calculatedVelocity = (rpm/60)*ticksPerRotation;
        rightFlywheelMotor.setVelocity(calculatedVelocity);
        leftFlywheelMotor.setVelocity(calculatedVelocity);
    }
    /**
     * returns a boolean telling whether the flywheel is at speed or not
     * @param speed the speed to check against the flywheel in rpm
     * @param error the amount of error of margin of rpm to still return true
     */
    public boolean flywheelIsSpedUp(int speed, double error){
        if(rightFlywheelMotor.getVelocity() < speed + error && rightFlywheelMotor.getVelocity() > speed){
            return true;
        }
        else{
            return false;
        }
    }




}
