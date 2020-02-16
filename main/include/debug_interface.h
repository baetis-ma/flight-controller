void debug_interface() {
   printf("%7.2fs %3d tune-> %c,%c/%6.3f  ", 0.001*esp_log_timestamp(), nsamp - lastsamp, axis, par, amt);
   if(axis == 'x') printf("PID gain %3.2f %3.2f %3.0f  I=%7.1f D=%5.2f  FusX=%6.2f  th=%4d  xPID= %3d %3d %3d",
        xPgain,xIgain,xDgain,xInt,xDer,xFusion,throttle,(int)(xPgain*xErr),(int)(xIgain*xInt),(int)(xDgain*xDer));
   if(axis == 'y') printf("PID gain %3.2f %3.2f %3.0f  I=%7.1f D=%5.2f  FusY=%6.2f  th=%4d  yPID= %3d %3d %3d",
        yPgain,yIgain,yDgain,yInt,yDer,yFusion,throttle,(int)(yPgain*yErr),(int)(yIgain*yInt),(int)(yDgain*yDer));
   if(axis == 'z') printf("PID gain %3.2f %3.2f %3.0f  I=%7.1f D=%5.2f  yawZ=%6.2f  th=%4d  zPID= %3d %3d %3d",
        zPgain,zIgain,zDgain,zInt,zDer,zErr,   throttle,(int)(zPgain*zErr),(int)(zIgain*zInt),(int)(zDgain*zDer));
   printf("  Motors  %4d %4d %4d %4d PID %3dx %3dy %3dz\n", 
        Motor1, Motor2, Motor3, Motor4, (int)xPIDout, (int)yPIDout, (int)zPIDout);
   lastsamp = nsamp;
   scanf("%c\n", &col);
   if(col!=0){
       if (col == 'M') amt = amt * 10; 
       if (col == 'm') amt = amt / 10; 
       if (col == 'a') { axis = axis + 1; if (axis > 'z') axis = 'x'; } 
       if (col == 'p') par = col; 
       if (col == 'i') {par = col; xInt=0; yInt=0; zInt=0; zGyro_Int=0; } 
             
       if (col == 'd') par = col; 
       if (col == 't') par = col; 
       if (col == 'T') par = col; 
       if (col == 'r') {throttle = 1000; xInt=0; xIgain=0; xPgain=0; xDgain=0;amt=0.1;
                        yInt=0; yIgain=0; yPgain=0; yDgain=0; zInt=0; zGyro_Int=0; zPgain=0; zDgain=0; zIgain=0;}
       if (par == 't' && col == '+') throttle = throttle + 1;
       if (par == 'T' && col == '+') throttle = throttle + 50;
       if (par == 't' && col == '-') throttle = throttle - 1;
       if (par == 'T' && col == '-') throttle = throttle - 50;
       if (axis == 'x' && par == 'p' && col == '+') xPgain = xPgain + amt; 
       if (axis == 'x' && par == 'i' && col == '+') {xIgain = xIgain + amt; xInt=0; yInt=0; }
       if (axis == 'x' && par == 'd' && col == '+') xDgain = xDgain + amt; 
       if (axis == 'x' && par == 'p' && col == '-') xPgain = xPgain - amt;
       if (axis == 'x' && par == 'i' && col == '-') {xIgain = xIgain - amt; xInt=0; yInt=0; }
       if (axis == 'x' && par == 'd' && col == '-') xDgain = xDgain - amt;
       if (axis == 'y' && par == 'p' && col == '+') yPgain = yPgain + amt; 
       if (axis == 'y' && par == 'i' && col == '+') {yIgain = yIgain + amt; xInt=0; yInt=0; }
       if (axis == 'y' && par == 'd' && col == '+') yDgain = yDgain + amt; 
       if (axis == 'y' && par == 'p' && col == '-') yPgain = yPgain - amt;
       if (axis == 'y' && par == 'i' && col == '-') {yIgain = yIgain - amt; xInt=0; yInt=0; }
       if (axis == 'y' && par == 'd' && col == '-') yDgain = yDgain - amt;
       if (axis == 'z' && par == 'p' && col == '+') {zPgain = zPgain + amt;  zGyro_Int = 0; }
       if (axis == 'z' && par == 'i' && col == '+') zIgain = 0; //zIgain + amt; 
       if (axis == 'z' && par == 'd' && col == '+') {zDgain = zDgain + amt;  zGyro_Int = 0; }
       if (axis == 'z' && par == 'p' && col == '-') {zPgain = zPgain - amt; zGyro_Int = 0; }
       if (axis == 'z' && par == 'i' && col == '-') zIgain = 0; //zIgain - amt;
       if (axis == 'z' && par == 'd' && col == '-') {zDgain = zDgain - amt; zGyro_Int = 0; }
       if (col == 's' ){ xPgain = 1.0; xIgain = 0.05; xDgain = 60; 
                         yPgain = 1.0; yIgain = 0.05; yDgain = 60; 
                         zPgain = 3; xInt=0; yInt=0; zGyro_Int=0;}
       if (col == 'C' ) cal = 1;
       if (col == 'c' ) {cal = 0; Motor1 = 1000;  Motor2 = 1000;  Motor3 = 1000;  Motor4 = 1000; }
       if (col == 'h' || col == '?'){
            printf("Options \n");
            printf("h or ?   - this message\n");
            printf("p, i, d  - select parameter to tune p i d \n");
            printf("M or m   - increase/decrease amt by factor of 10\n");
            printf("+ or -   - increase/decrease parmameter by amt \n");
            printf("T or t   - select throttle adjment T +/- 50, t +/- 1 \n");
            printf("C or c   - select for esc cal C max throt c = min \n");
            printf("s        - load stanard default values \n");
            printf("r        - reset throttle to 1000 \n");
       }
   }
   col = 0;
}
