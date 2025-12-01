package org.firstinspires.ftc.teamcode.Utilities;

import androidx.annotation.NonNull;

import java.util.NavigableMap;
import java.util.TreeMap;

public class RpmLookupTable {
    NavigableMap<Integer,Integer> distanceAndRpm = new TreeMap<>();
    public RpmLookupTable(){

        //k is distance and v is rpm
        distanceAndRpm.put(55,  2450);
        distanceAndRpm.put(60, 2500);
        distanceAndRpm.put(65,2600);
        distanceAndRpm.put(70, 2650);
        distanceAndRpm.put(75, 2775);
        distanceAndRpm.put(80, 2850);
        distanceAndRpm.put(85, 2900);
        distanceAndRpm.put(90,3050);
        distanceAndRpm.put(95,3150);
        distanceAndRpm.put(105, 3250);
        distanceAndRpm.put(110, 3300);
        distanceAndRpm.put(117, 3550);
        distanceAndRpm.put(123, 3700);
        distanceAndRpm.put(130, 3800);
        distanceAndRpm.put(135, 3850);
        distanceAndRpm.put(140, 3900);
        distanceAndRpm.put(145, 3950);
        distanceAndRpm.put(150, 4000);
        //single wheel
        /*distanceAndRpm.put(144, 4350);
        distanceAndRpm.put(136, 4250);
        distanceAndRpm.put(128, 4100);
        distanceAndRpm.put(118, 4000);
        distanceAndRpm.put(107, 3850);
        distanceAndRpm.put(97, 3700);
        distanceAndRpm.put(85, 3650);
        distanceAndRpm.put(64, 3450);*/


    }
    public int getRpm(@NonNull int target){
        Integer floorKey = distanceAndRpm.floorKey(target);
        Integer ceilingKey = distanceAndRpm.ceilingKey(target);

        if(ceilingKey == null){
            return distanceAndRpm.get(floorKey);
        }
        else if(floorKey == null){
            return distanceAndRpm.get(ceilingKey);
        }
        if(Math.abs(target - floorKey) <= Math.abs(target - ceilingKey)){
            return distanceAndRpm.get(floorKey);
        }
        else{
            return distanceAndRpm.get(ceilingKey);
        }




    }
}
