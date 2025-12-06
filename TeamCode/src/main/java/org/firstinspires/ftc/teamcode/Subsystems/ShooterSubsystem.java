package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Utilities.RpmLookupTable;


public class ShooterSubsystem {
    Config config;
    public DcMotorEx flywheel;
    DcMotor starboardBelt;
    DcMotor portBelt;
    RpmLookupTable table;
    Servo gate;
    DcMotor intake;
    int targetRpm;

    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();

// In your init() or start() method

    public ShooterSubsystem(HardwareMap hwMap) {
        config = new Config();
        flywheel = hwMap.get(DcMotorEx.class, config.MainFlywheelMotorName);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //belts
        starboardBelt = hwMap.get(DcMotor.class, config.sbBeltName);
        portBelt = hwMap.get(DcMotor.class, config.portBeltName);
        //directions
        starboardBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        portBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        //runmode
        starboardBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        portBelt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        table = new RpmLookupTable();
        gate = hwMap.get(Servo.class, config.GateServoName);
        intake = hwMap.get(DcMotor.class, "intake");
        kP = 0.0001;
        kI = 0.0001;
        kD = 0;


    }

    /**
     * @param rpm is the desired speed in rpm
     */

    public void setFlywheelVelocity(int rpm) {
        flywheel.setVelocity(-(rpm / 60) * config.ticksPerRevFlywheel);

    }

    /**
     * @return current rpm of the flywheel
     */

    public double getRpm() {
        return (-flywheel.getVelocity() / 28) * 60;

    }

    /**
     * spins belt
     *
     * @param power is the power from -1 to 1
     */
    public void spinBelt(double power) {
        starboardBelt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        starboardBelt.setPower(power);
        portBelt.setPower(power);
    }

    /**
     * sets the target position
     */
    public void setTargetPosition(int pose) {
        starboardBelt.setTargetPosition(starboardBelt.getCurrentPosition() + pose);
        starboardBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateBeltPosition() {
        if(beltIsBusy()) {
            starboardBelt.setPower(1);
        }
            portBelt.setPower(starboardBelt.getPower()*0.3);

    }
    public boolean beltIsBusy(){
        return starboardBelt.isBusy();
    }
    public double beltPose(){
        return starboardBelt.getCurrentPosition();
    }
    public void setRpmUsingTable(int distance) {
        setFlywheelVelocity(-table.getRpm(distance));
    }

    public int flywheelGetRpmFromTable(int distance) {
        return table.getRpm(distance);
    }

    public void setServoPosition(double position) {
        gate.setPosition(position);
    }

    public void spinIntake(double power) {
        intake.setPower(power);
    }

    public int getPositionOfBelts() {
        return starboardBelt.getCurrentPosition();
    }
    public void stopBelt(){
        portBelt.setPower(0);
    }

    public void resetBeltTicks() {
        starboardBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetRpm(int giventargetRpm) {
        targetRpm = giventargetRpm;
    }
    public void calculate() {
        // Calculate error
        double error =  targetRpm - getRpm();

        // Proportional term
        double proportional = error * kP;

        // Integral term
        integralSum += (error * timer.seconds());
        // Implement integral windup mitigation here if needed

        // Derivative term
        double derivative = (error - lastError) / timer.seconds();

        // Update last error and reset timer for next iteration
        lastError = error;
        timer.reset();

        // Calculate total output
        flywheel.setPower(-(proportional + integralSum + derivative)/10);
    }
}


















