package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Configs.Config;
import org.firstinspires.ftc.teamcode.Sensors.OdoPods;

import java.util.List;


public class Aimbots {
    public Limelight3A limelight;
    public boolean isBlue;
    public OdoPods pods;
    public double targetX;
    public double targetY;
    public double AdjacentSide;
    public double OppositeSide;
    Config config = new Config();
    public double CVertX, CVertY;
    public double AvertX, AvertY;
    public double BvertX, BvertY;
    int givenAlliance;
    LLResult result;

    public Aimbots(@NonNull int alliance, OdoPods Givenpods, HardwareMap hardwareMap) {
        pods = Givenpods;
        if (alliance == config.RedAlliance) {
            AvertX = config.RedAllianceTargetX;
            AvertY = config.RedAllianceTargetY;
        } else if (alliance == config.BlueAlliance) {
            AvertX = config.BlueAllianceTargetX;
            AvertY = config.BlueAllianceTargetY;
        }
        givenAlliance = alliance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.resetDeviceConfigurationForOpMode();
        limelight.start();
    }

    //    a
    //  b_____a
    //  |  x/    x = 90 - acos(a/h)
    // o|  / h
    //  | /
    //  |/
    //  c
    public void update() {
        CVertX = pods.getX();
        CVertY = pods.getY();
        BvertY = CVertY;
        BvertX = pods.getX();
        result = limelight.getLatestResult();
        pods.update();
    }

    /**
     * @return distance from the given target previously selected
     * based on alliance picked
     */
    public double calculateSideLengthUsingPods() {
        return Math.hypot((AvertX - CVertX), (AvertY - CVertY));
    }

    /**
     * @return ideal robot angle to face target
     */
    public double getIdealAngle() {
        double toReturn = 0;
        if (Math.toDegrees(Math.acos(((CVertX - AvertX) / calculateSideLengthUsingPods()))) > 180) {
        } else {
            toReturn = Math.toDegrees(Math.acos(((CVertX - AvertX) / calculateSideLengthUsingPods()))) - 150;
        }

        return -Math.toDegrees(Math.acos(((CVertX - AvertX) / calculateSideLengthUsingPods()))) + 90;

    }

    public double getDistanceUsingLL() {
        double targetHeight = 29.75;
        double LLHeight = 6.75;
        return (targetHeight - LLHeight) / Math.tan(Math.toRadians(20-result.getTy()));
    }

    public void stopLL() {
        limelight.stop();
    }

    public void switchPipeline(@NonNull int givenPipeline) {
        limelight.pipelineSwitch(givenPipeline);
    }

    public void startLL() {
        limelight.start();
    }

    public double getHeadingErrorLL() {
        return result.getTx();
    }

    public void correctWithHeadingError(double speed) {
        pods.holdHeading(pods.getHeading() - getHeadingErrorLL(), speed);
    }
    public boolean LLstatusIsValid() {
        return result.isValid();
    }
    public int LLgetApriltagID(){
        int toReturn = 0;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for(LLResultTypes.FiducialResult fr : fiducialResults){
            toReturn = fr.getFiducialId();
        }
        return toReturn;
    }

}
