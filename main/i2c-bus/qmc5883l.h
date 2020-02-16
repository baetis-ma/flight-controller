
void qmc5883_init() {
    i2c_write(QMC5883L_I2C_ADDR, 0x09, 0x01); 
    i2c_write(QMC5883L_I2C_ADDR, 0x0a, 0x00); 
    i2c_write(QMC5883L_I2C_ADDR, 0x0b, 0x01); 
}

int qmc5883_read () {
    int xmax = 9952; int xmin = -1437;
    int ymax = 3737; int ymin = -7930;
    int zmax = 3622; int zmin = -8395;
    float caloff = -2.3;   
    uint8_t regdata[8];
    i2c_read(QMC5883L_I2C_ADDR, 0x00, regdata, 6); 

    //for (int a =0; a<6;a++)printf("0x%02x ",regdata[a]);// printf("\n");
    
    int x = 256 * regdata[1] + regdata[0]; if(regdata[1]>=128) x = x - (1 << 16);
    int y = 256 * regdata[3] + regdata[2]; if(regdata[3]>=128) y = y - (1 << 16);
    int z = 256 * regdata[5] + regdata[4]; if(regdata[5]>=128) z = z - (1 << 16);

    float xmag = (float) (x - 0.5*(xmax+xmin));
    float ymag = (float) (y - 0.5*(ymax+ymin));
    float zmag = (float) (z - 0.5*(zmax+zmin));
    xmag = (float) (xmag / 0.5*(xmax-xmin));
    ymag = (float) (ymag / 0.5*(ymax-ymin));
    zmag = (float) (zmag / 0.5*(zmax-zmin));
    float phi = 90 + 57.3 * atan(ymag/xmag);
    if (phi < xmag) phi = phi + 180;
    int phii = (int) (phi - caloff);
    phii = (360 + phii) % 360;
    //printf("  phi = %5.3f\n",phi);
    return ( phii );
}

