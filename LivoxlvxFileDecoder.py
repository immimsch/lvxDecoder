import os, struct, csv

class LivoxlvxFileDecoder:
    def __init__ (self, lvxFileName):
        self.lvxFileName = lvxFileName
        self.deviceType = { 0 : 'LiDar Hub', 1 : 'Mid-40/Mid-100', 2 : 'Tele-15', 3 : 'Horizon'}
        self.LivoxFileRead()

    def LivoxFileRead(self):
        Idx = 28
        rf = open(self.lvxFileName, 'rb')
        d = rf.read()
        fileSize = len(d)
        DeviceCount = d[Idx]
        Idx += 1
        list1 = []

        for Device in range(DeviceCount):
            list1.append(['LidarSN', d[Idx:Idx+16].decode()])
            list1.append(['Device Type', self.deviceType[d[Idx+33]]])
            Idx = Idx + 59
        
        while (Idx < fileSize) :
            nxt = int.from_bytes(d[Idx+8:(Idx+16)],'little')
            Idx = Idx + 24
            previousDataType = 100 # A random number out of dtype range.
            writeBefore = False
            
            while Idx < nxt:            
                dtype = d[Idx+10]
                Idx = Idx + 19

                if dtype==6:                                         
                    # IMU Information.
                    gyro_X = struct.unpack('f', d[Idx:Idx+4])
                    gyro_Y = struct.unpack('f', d[Idx+4:Idx+8])
                    gyro_Z = struct.unpack('f', d[Idx+8:Idx+12])
                    acc_X = struct.unpack('f', d[Idx+12:Idx+16])
                    acc_Y = struct.unpack('f', d[Idx+16:Idx+20])
                    acc_Z = struct.unpack('f', d[Idx+20:Idx+24])  
                    list_tmp = ([dtype, gyro_X[0], gyro_Y[0], gyro_Z[0], acc_X[0], acc_Y[0], acc_Z[0]])
                    if not gyro_X[0]+gyro_Y[0]+gyro_Z[0]+acc_X[0]+acc_Y[0]+acc_Z[0] == 0 and previousDataType != dtype:
                        list1.append(['Data Type', 'gyro_X', 'gyro_Y', 'gyro_Z', 'acc_X', 'acc_Y', 'acc_Z'])     
                        list1.append(list_tmp)                
                    Idx = Idx + 24
                    previousDataType = dtype
                elif dtype==0:                 
                    # Cartesian Coordinate System, Single Return, Mid Only.
                    for i in range(100):
                        X = int.from_bytes(d[Idx:Idx+4],'little', signed=True)
                        Y = int.from_bytes(d[Idx+4:Idx+8],'little', signed=True)
                        Z = int.from_bytes(d[Idx+8:Idx+12],'little', signed=True)
                        if not X+Y+Z==0 and X+Y+Z<1e6:
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'X(mm)', 'Y(mm)', 'Z(mm)', 'Reflectivity']) 
                            writeBefore = True                           
                            list_tmp = ([dtype, X, Y, Z, d[Idx+12]])                       
                            list1.append(list_tmp)
                        Idx = Idx + 13    
                    previousDataType = dtype
                    writeBefore = False
                elif dtype==1:
                    # Spherical Coordinate System, Single Return, Mid Only.
                    for i in range(100):
                        depth = int.from_bytes(d[Idx:Idx+4],'little', signed=True)
                        theta = int.from_bytes(d[Idx+4:Idx+6],'little', signed=False)
                        phi = int.from_bytes(d[Idx+6:Idx+8],'little', signed=False)
                        if not depth+theta+phi==0 and depth+theta+phi<1e6:
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'depth(mm)', 'theta(0.01 degree)', 'phi(0.01 degree)', 'Reflectivity'])    
                            writeBefore = True                          
                            list_tmp = ([dtype, depth, theta, phi, d[Idx+8]])
                            list1.append(list_tmp)
                        Idx = Idx + 9
                    previousDataType = dtype
                    writeBefore = False
                elif dtype==2:                  
                    # Cartesian Coordinate System, Single Return.
                    for i in range(96):
                        X = int.from_bytes(d[Idx:Idx+4],'little', signed=True)
                        Y = int.from_bytes(d[Idx+4:Idx+8],'little', signed=True)
                        Z = int.from_bytes(d[Idx+8:Idx+12],'little', signed=True)
                        if not X+Y+Z==0 and X+Y+Z<1e6:
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'X(mm)', 'Y(mm)', 'Z(mm)', 'Reflectivity'])     
                            writeBefore = True                            
                            list_tmp = ([dtype, X, Y, Z, d[Idx+12]])
                            list1.append(list_tmp)
                        Idx = Idx + 14
                    previousDataType = dtype
                    writeBefore = False
                elif dtype==3: 
                    # Spherical Coordinate System, Single Return.
                    for i in range(96):
                        depth = int.from_bytes(d[Idx:Idx+4],'little', signed=True)
                        theta = int.from_bytes(d[Idx+4:Idx+6],'little', signed=False)
                        phi = int.from_bytes(d[Idx+6:Idx+8],'little', signed=False)
                        if not depth+theta+phi==0 and depth+theta+phi<1e6:
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'depth(mm)', 'theta(0.01 degree)', 'phi(0.01 degree)', 'Reflectivity'])                          
                            writeBefore = True
                            list_tmp = ([dtype, depth, theta, phi, d[Idx+8]])
                            list1.append(list_tmp)
                        Idx = Idx + 10
                    previousDataType = dtype
                    writeBefore = False
                elif dtype==4:
                    # Cartesian Coordinate System, Dual Return.
                    for i in range(48):
                        X_1 = int.from_bytes(d[Idx:Idx+4],'little', signed=True)
                        Y_1 = int.from_bytes(d[Idx+4:Idx+8],'little', signed=True)
                        Z_1 = int.from_bytes(d[Idx+8:Idx+12],'little', signed=True)
                        X_2 = int.from_bytes(d[Idx+14:Idx+18],'little', signed=True)
                        Y_2 = int.from_bytes(d[Idx+18:Idx+22],'little', signed=True)
                        Z_2 = int.from_bytes(d[Idx+22:Idx+26],'little', signed=True)                    
                        if not X_1+Y_1+Z_1+X_2+Y_2+Z_2==0 and X_1+Y_1+Z_1+X_2+Y_2+Z_2<1e6:  
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'X_1(mm)', 'Y_1(mm)', 'Z_1(mm)', 'Reflectivity_1', 'X_2(mm)', 'Y_2(mm)', 'Z_2(mm)', 'Reflectivity_2'])
                                writeBefore = True
                                list_tmp = ([dtype, X_1, Y_1, Z_1, d[Idx+12], X_2, Y_2, Z_2, d[Idx+26]])
                            list1.append(list_tmp)
                        Idx = Idx + 28
                    previousDataType = dtype
                    writeBefore = False
                elif dtype==5:
                    # Spherical Coordinate System, Single Return.
                    for i in range(48):
                        theta = int.from_bytes(d[Idx:Idx+2],'little', signed=False)
                        phi = int.from_bytes(d[Idx+2:Idx+4],'little', signed=False)
                        depth_1 = int.from_bytes(d[Idx+4:Idx+8],'little', signed=True)
                        depth_2 = int.from_bytes(d[Idx+10:Idx+14],'little', signed=True)
                        if not depth_1+depth_2+theta+phi==0 and depth_1+depth_2+theta+phi<1e6:
                            if previousDataType != dtype and not writeBefore:
                                list1.append(['Data Type', 'theta(0.01 degree)', 'phi(0.01 degree)', 'depth_1(mm)', 'Reflectivity_1', 'depth_2(mm)', 'Reflectivity_2'])
                                writeBefore = True  
                                list_tmp = ([dtype, depth_1, theta, phi, d[Idx+8], depth_2, d[Idx+14]])
                            list1.append(list_tmp)
                        Idx = Idx + 16
                    previousDataType = dtype
                    writeBefore = False
            
        rf.close
        return list1

        
if __name__ == '__main__':
    c = LivoxlvxFileDecoder(r'YourlvxFilePathHere')
    dataList = c.LivoxFileRead()
    for l in dataList:
        print(l)


