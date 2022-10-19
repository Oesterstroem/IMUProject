#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

void I2C_Init()
{
  Wire.begin();
}


void Calc_Offsets()
{
  for(int i=0;i<32;i++)    
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]+=GRAVITY*SENSOR_SIGN[5];
  
	delay(20);
	for(int n = 0; n < SAMPLE_NUMBER; n++)
	{ 
		Read_Gyro();
		for(int y = 0; y < 3; y++)
		{
			if((int)AN[y]-AN_OFFSET[y] > NOISE[y]) 
				NOISE[y] = (int) AN[y] - AN_OFFSET[y]; 

			else if((int) AN[y] - AN_OFFSET[y] <-NOISE[y]) 
				NOISE[y]=-(int) AN[y] - AN_OFFSET[y]; 

		} 
		for(int y = 0; y < 3; y++)
		{
			NOISE[y] = NOISE[y] / 100; 
			//Serial.print("Noise: ");Serial.println(NOISE[y]);
		}
	}
}

void Gyro_Init()
{
  gyro.init();
  gyro.writeReg(L3G_CTRL_REG4, 0x20); 
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); 
}

void Read_Gyro()
{
  gyro.read();
  
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
  compass.init();
  compass.enableDefault();
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); 
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); 
      break;
    default: 
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); 
  }
}


void Read_Accel()
{
  compass.readAcc();
  
  AN[3] = compass.a.x >> 4; 
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{
 
}

void Read_Compass()
{
  compass.readMag();
  
  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}
