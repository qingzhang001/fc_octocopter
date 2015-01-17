#include <mega168.h>
#include <math.h>

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.0005
#define Kp_YAW 1.2
#define Ki_YAW 0.0005
#define GRAVITY 258
#define drift_corr 1

float sampling_time()          //mengembalikan nilai lamanya eksekusi program dalam detik
{
       
static int timer,timer_old;
float sample_time;
int msb_timer, lsb_timer,count;
        
       
        msb_timer=TCNT1H;
        lsb_timer=TCNT1L;
        timer=(((int)(msb_timer)<<8) | lsb_timer);
        if(timer_old>timer)
        {
        sample_time=0.0032*((65535-timer_old)+timer);
        }
        else
        {
        sample_time=0.0032*(timer-timer_old);
        };
        timer_old=timer;
   
        return sample_time;
}


void matrix_update()
{     

        float temp[3][3]; 
        float temp2[3][3]; 

        char i,j;
    vector_add(&delta_temp[0],&gyro_vector[0],&I_part[0]);  
    vector_add(&delta[0],&delta_temp[0],&P_part[0]);    
      
     //nilai update untuk AHRS dengan koreksi drift
           update_matrix[0][0]=0;
           update_matrix[0][1]=-dt*delta[2];
           update_matrix[0][2]=dt*delta[1];
           update_matrix[1][0]=dt*delta[2];
           update_matrix[1][1]=0;
           update_matrix[1][2]=-dt*delta[0];
           update_matrix[2][0]=-dt*delta[1];
           update_matrix[2][1]=dt*delta[0];
           update_matrix[2][2]=0;
           
     //nilai update untuk AHRS ranpa koreksi drift
           update_matrix2[0][0]=0;
           update_matrix2[0][1]=-dt*gyro_vector[2];
           update_matrix2[0][2]=dt*gyro_vector[1];
           update_matrix2[1][0]=dt*gyro_vector[2];
           update_matrix2[1][1]=0;
           update_matrix2[1][2]=-dt*gyro_vector[0];
           update_matrix2[2][0]=-dt*gyro_vector[1];
           update_matrix2[2][1]=dt*gyro_vector[0];
           update_matrix2[2][2]=0;
 
     
     matrix_multiply(CBN_matrix,update_matrix,temp );
     matrix_multiply(CBN_matrix2,update_matrix2,temp2 );
     
    //update matrix CBN (koreksi drift)
     for(i=0;i<3;i++)
     {
       for (j=0;j<3;j++)
       {
                CBN_matrix[i][j]+=temp[i][j];
       }
     
     };
     
     //update matrixs CBN (tanpa koreksi drift)
     for(i=0;i<3;i++)
     {
       for (j=0;j<3;j++)
       {
                CBN_matrix2[i][j]+=temp2[i][j];
       }
     
     };

     
}

void renormalisation()
{
        float error;
        float temp[3][3];
        float renorm;
        
        
        error = -0.5*vector_dotproduct(&CBN_matrix[0][0],&CBN_matrix[1][0]); //dot product dari axis X dan Y 
        vector_scale(&temp[0][0],&CBN_matrix[1][0],error);
        vector_scale(&temp[1][0],&CBN_matrix[0][0],error);
        
        vector_add(&temp[0][0],&CBN_matrix[0][0],&temp[0][0]);
        vector_add(&temp[1][0],&CBN_matrix[1][0],&temp[1][0]);
        
        vector_crossproduct(&temp[2][0],&temp[0][0],&temp[1][0]);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[0][0],&temp[0][0])));
        vector_scale(&CBN_matrix[0][0],&temp[0][0],renorm);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[1][0],&temp[1][0])));
        vector_scale(&CBN_matrix[1][0],&temp[1][0],renorm);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[2][0],&temp[2][0])));
        vector_scale(&CBN_matrix[2][0],&temp[2][0],renorm);
 
        error = -0.5*vector_dotproduct(&CBN_matrix2[0][0],&CBN_matrix2[1][0]); //dot product dari axis X dan Y 
        vector_scale(&temp[0][0],&CBN_matrix2[1][0],error);
        vector_scale(&temp[1][0],&CBN_matrix2[0][0],error);
        
        vector_add(&temp[0][0],&CBN_matrix2[0][0],&temp[0][0]);
        vector_add(&temp[1][0],&CBN_matrix2[1][0],&temp[1][0]);
        
        vector_crossproduct(&temp[2][0],&temp[0][0],&temp[1][0]);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[0][0],&temp[0][0])));
        vector_scale(&CBN_matrix2[0][0],&temp[0][0],renorm);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[1][0],&temp[1][0])));
        vector_scale(&CBN_matrix2[1][0],&temp[1][0],renorm);
        
        renorm=0.5*(3-(vector_dotproduct(&temp[2][0],&temp[2][0])));
        vector_scale(&CBN_matrix2[2][0],&temp[2][0],renorm);

        
}


void drift_correction()
{
        char i;
        float mag_heading_x;
        float mag_heading_y;
        float errorCourse;
        
          static float Scaled_P_part[3];
    
          static float Scaled_I_part[3];
          static float old_error;
      
    
         //Koreksi roll dan pitch 
          Accel_magnitude = sqrt(acc_vector[0]*acc_vector[0] + acc_vector[1]*acc_vector[1] + acc_vector[2]*acc_vector[2]);
          Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
          temp= 1 - (Accel_magnitude*Accel_magnitude);
          
          Accel_weight = constrain(1 - 2*abso(temp),0,1); 
          vector_crossproduct(&error_rollpitch[0],&acc_vector[0],&CBN_matrix[2][0]); //adjust the ground of reference
          
           //bagian P
           vector_scale(P_part,&error_rollpitch[0],Kp_ROLLPITCH*Accel_weight*-1);
   
               
           //bagian I
           vector_scale(&Scaled_I_part[0],&error_rollpitch[0],-1*Ki_ROLLPITCH*Accel_weight);
           vector_add(I_part,I_part,Scaled_I_part);
           for(i=0;i<3;i++)
           {
                if(I_part[i]>0.02){I_part[i]=0.02;};
                if(I_part[i]<-0.02){I_part[i]=-0.02;};
           };     
          
            //Koreksi Yaw 
            Compass_Heading();
            mag_heading_x = cos(MAG_Heading);
            mag_heading_y = sin(MAG_Heading);
            errorCourse=(CBN_matrix[0][0]*mag_heading_y) - (CBN_matrix[1][0]*mag_heading_x); 

            vector_scale(error_yaw,&CBN_matrix[2][0],errorCourse); 
            
            //bagian P  
             vector_scale(&Scaled_P_part[0],&error_yaw[0],Kp_YAW);
             vector_add(P_part,P_part,Scaled_P_part);
             
            //Bagian I
          // vector_scale(&P_part[0],&error_yaw[0],Kp_YAW);
            vector_scale(&Scaled_I_part[0],&error_yaw[0],Ki_YAW);
            vector_add(I_part,I_part,Scaled_I_part);

}

float constrain(float a, float b,float c)
{
        if (a<b)
        {return b;}
        else if (a>c)
        {return c;}
        else return a;

}

float abso(float a)
{
        if (a<0) {return (-1*a);}
        else return a;
}

void Compass_Heading()     // mengembalikan nilai arah medan magnet dalam radian

{
        
         float cos_roll;
         float sin_roll;
         float cos_pitch;
         float sin_pitch;
         float MAG_X;
         float MAG_Y;
 
          cos_roll = cos(roll);
          sin_roll = sin(roll);
          cos_pitch = cos(pitch);
          sin_pitch = sin(pitch);
          MAG_X = mag_x*cos_pitch+mag_y*sin_roll*sin_pitch+mag_z*cos_roll*sin_pitch;
          MAG_Y = mag_y*cos_roll-mag_z*sin_roll;
                    
          MAG_Heading = atan2(-MAG_Y,MAG_X);
}

void Euler_angle()             //menghitung nilai sudut2 euler berdasarkan matrix CBN
{
        pitch = -asin(CBN_matrix[2][0]);
        roll = atan2(CBN_matrix[2][1],CBN_matrix[2][2]);
        yaw = atan2(CBN_matrix[1][0],CBN_matrix[0][0]);  
        
        pitch2 = -asin(CBN_matrix2[2][0]);
        roll2 = atan2(CBN_matrix2[2][1],CBN_matrix2[2][2]);
        yaw2 = atan2(CBN_matrix2[1][0],CBN_matrix2[0][0]);
        
}
