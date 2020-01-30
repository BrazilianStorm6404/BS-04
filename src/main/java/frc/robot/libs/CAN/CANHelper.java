/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.can;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

/**
 * Add your docs here.
 */
public class CANHelper {

    //#region INSTANTIATION
    private CAN CANSender;
    private CANData data;
    private byte[] CANBytes = new byte[8];
    //#endregion

    //#region CONSTRUCTOR
    public CANHelper(String deviceID) {
        CANSender = new CAN(Integer.parseInt(deviceID, 16));
        data = new CANData();
        for(byte CANbyte : CANBytes){
            CANbyte = (byte) 0;
        }
    }
    //#endregion

    //#region WRITE DATA
    public void writeData(String id, int time) {
        CANSender.writePacketRepeating(CANBytes, Integer.parseInt(id, 16), time);
    }
    //#endregion

    //#region READ DATA
    public byte[] readData(String id) {
        boolean state = CANSender.readPacketLatest(Integer.parseInt(id, 16), this.data); 
        return this.data.data;
    } 
    //#endregion

    //#region GET CANBYTES
    public byte[] getCANBytes() {
        return CANBytes;
    }
    //#endregion

    //#region SET CANBYTES
    public void setCANBytes(byte[] CANBytes) {
        this.CANBytes = CANBytes;
    }
    //#endregion
}