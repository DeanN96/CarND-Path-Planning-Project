## Reflection 

1. find lane where the car is running 

            for(int i = 0; i < sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                int carlane = -1;
              
                if (d < 12 && d > 8 ){
                    carlane = 2;
                } else if (d < 8 && d > 4){
                    carlane = 1;
                } else if (d < 4 && d > 0){
                    carlane = 0;
                }
              
                if (carlane == -1){
                    continue;
                }
    
 2. Next we check the car speed and direction. If there is a car in my lane record the speed at which is it is driving. If the car is on a different lane, and the position is between (-15, 30), that is the car which is stored. 
 
                long double v_x = sensor_fusion[i][3];
                long double v_y = sensor_fusion[i][4];
                long double checkspeed = sqrt(v_x*v_x+v_y*v_y);
                long double checkcars = sensor_fusion[i][5];
                checkcars = checkcars + ((long double)prevsize * .02 * checkspeed);
               
                if (carlane == lane){ // car is in my lane
                    if(30 > ((checkcars - car_s)) && (car_s < checkcars)){
                        carahead = true;
                        if(sclosest > (checkcars - car_s)){
                            sclosest = checkcars;
                            carvel = 2.24*checkspeed;
                        }
                    }
                } else if(carlane == (lane + 1) ){
                    if(-15 < ((checkcars - car_s)) && (30 > (checkcars - car_s))) {
                        carright = 1;
                    }                  

                } else if(carlane  == (lane - 1)){
                    if(-15 > ((checkcars - car_s)) && (30 > (checkcars - car_s))){
                        carleft = 1;
                    }
                }
            }
            
3. When there is a car infront of me, the car will change lanes if the way is free. If the lane is not free then the car will not change lanes. Finally, if the car is not on the centre lane, it will return to that lane

            long double d_vel = 0.0;
            const long double MAXSPEED = 49.5;
            const long double MAXACC = .224;
            long double targetspeed = MAXSPEED;
            
            
            if(targetspeed <= carvel){   
            }
          	else {
              targetspeed = carvel;
            }
            
            long double veldiff = targetspeed - car_speed;
            d_vel = 0.1*MAXACC * veldiff;
            if(d_vel < MAXACC){
                d_vel = -MAXACC;
            } else if(d_vel > -MAXACC){
                d_vel = MAXACC;
            }
           

