void flight_control()
{
   float err_alt,err_pitch,err_roll,err_yaw;
   static float acc_alt,acc_pitch,acc_roll,acc_yaw;
   char Kp_alt = 1118, Kp_pitch = 788 , Kp_roll = 1479;
   int Kp_yaw=3200,Kd_yaw=6000; 
   char Kd_alt=896, Kd_pitch=523 , Kd_roll=481;
   char Ki_alt=1, Ki_roll=1, Ki_pitch=1, Ki_yaw=1; 
   char Kii_alt=12, Kii_roll=15, Kii_pitch=13, Kii_yaw=15;
   float temp1,temp2,temp3,temp4;
   float AW_alt,AW_roll,AW_pitch,AW_yaw;
   
   err_alt = alt_sp.nilai-alt.nilai; 
   err_roll = rll_sp.nilai-rll.nilai;
   err_pitch = ptc_sp.nilai-ptc.nilai;
   err_yaw = y_sp.nilai-y.nilai;
   
   acc_alt = acc_alt+ Ki_alt*err_alt*dt+AW_alt;
   acc_roll = acc_roll+ Ki_roll*err_roll*dt+AW_roll; 
   acc_pitch = acc_pitch+ Ki_pitch*err_pitch*dt+AW_pitch;
   acc_yaw = acc_yaw+ Ki_yaw*err_yaw*dt+AW_yaw;
     
   temp1 =   (err_alt*Kp_alt)          - (alt_dot.nilai*Kd_alt)              + acc_alt;
   temp2 =  (err_roll*Kp_roll)        - (rll_dot.nilai*Kd_roll)              + acc_roll;
   temp3 =    (err_pitch*Kp_pitch)      - (ptc_dot.nilai*Kd_pitch )         + acc_pitch ;
   temp4 =   (err_yaw*Kp_yaw)          - (y_dot.nilai*Kd_yaw)                + acc_yaw;

   if(temp1>150){U1=150;}
   else if(temp1<-150){U1=-150;}
   else {U1=temp1;}
   AW_alt = Kii_alt*(U1-temp1);
   
   if(temp2>50){U2=50;}
   else if(temp2<-50){U2=-50;}
   else {U2=temp2;}
   AW_roll = Kii_roll*(U2-temp2);
   
   if(temp3>50){U3=50;}
   else if(temp3<-50){U3=-50;}
   else {U3=temp3;}
   AW_pitch = Kii_pitch*(U3-temp3);
   
   if(temp4>200){U4=200;}
   else if(temp4<-200){U4=-200;}
   else {U4=temp4;}
   AW_yaw = Kii_yaw*(U4-temp4);
} //test
