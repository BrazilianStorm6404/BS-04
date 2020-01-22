/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.math.shot;

import frc.robot.Constants;

/**
 * Used to validate the distance to launch
 */
public class WithoutAirDrag {
    private boolean getPosibilite(double distanceToTry){
        return (distanceToTry>=Constants.minDistToShot & distanceToTry<=Constants.maxDistToShot);
    }
    public Double getAngleToPitch(double distanceToTry){
        if(getPosibilite(distanceToTry))
            return null;
        else{
            return 0.0;//Coloca a funcao dps
        }
    }
}
