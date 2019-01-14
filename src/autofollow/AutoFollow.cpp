#include "AutoFollow.h"
#include <iostream>

AutoFollow::AutoFollow(){
    if(!find.initial()){
        cout<<"findplate initial fail"<<endl;
        exit(-1);
    }

#ifdef SERIAL
 	 if(!serial.open_port()) {
        cout << "open port fail" << endl;
        exit(-1);
    	}
#endif // !end SERIAL
    if(!capture.open(0)){
        if(!capture.open(1)) {
            cout << "capture open fail" << endl;
            exit(-1);
        }
    }
    waitKey(200);
    cout<<"initial OK!!!"<<endl;
}

void AutoFollow::run() {
/*   while(waitKey(1)!=27){
        if(state==CPark){
            cout<<"state=  CPark"<<endl;
            doPark();
        }
        else if(state==Nothing) {
            cout<<"state=  Nothing"<<endl;
            doNothing();
        }
        else{
            cout<<"state error"<<endl;
            exit(-1);
        }
    }
*/
	buffpnt=0;
	int flag1=0;
	int flag2=0;
 	double average;
	int sum;

	while(true)
	{
		capture >> srcImage;
		
		if (!find.getlicense(srcImage, point, theta)) 
		{
          	flag1++;
           	if(flag1>=5)
			{
	            state=Nothing; 
				buffpnt=0;
				flag1=0;
	        }
			else
				;
			cout<<"Can't findlicense!"<<endl;
            goto Send;
			
        }	
		else
		{
cout<<"point.y= "<<point.y<<endl;
			flag1=0;
			if(state == Nothing || state == FirstTime)
			{
				buff_y[buffpnt]=point.y;
				buff_z[buffpnt]=point.x;
				buff_theta[buffpnt]=theta.y;
				buffpnt++;
if(buffpnt == 16)
{buffpnt =0;
}
				//buffpnt%16;
				state=Forward;
			}
			else
			{
				if(buffpnt==0)
				{	
					sum=0;
					for(int i=0;i<16;i++)
					{
						sum+=buff_y[i];
					}
					average=sum/16;
				}
				else
				{
					sum=0;
					for(int i=0;i<=buffpnt;i++)
					{
						sum+=buff_y[i];
					}	
					average=sum/(buffpnt+1);
				}	
				if(point.y<(average + 400) && point.y>(average - 400) && point.y>0)
				{
					flag2=0;
					buff_y[buffpnt]=point.y;
					buff_z[buffpnt]=point.x;
					buff_theta[buffpnt]=theta.y;
					
//					buffpnt%16;
					state=Forward;
				}
				else
				{
					cout<<"Data is wrong!"<<endl;
					flag2++;
					if(flag2>=5){
                			state=Nothing; 
					buffpnt=0;
					flag2=0;
            		}
				}
			}
		}
		//waitKey(10);
		Send:		
		int n=serial.receive();
        	if(n>0 && (serial.buf[0]=='c'))
		{
			cout<<"**********Sending Data!!!**********"<<endl;
			cout<<"buffpnt = "<<buffpnt<<endl;
			if(state==Forward)
			{
				int temp=buff_y[buffpnt];
				if(buff_y[buffpnt]*4/3>300)
				{
					temp=(int)buff_y[buffpnt]*4/3-300;
				}
				else
					temp=0;
				if (abs(buff_theta[buffpnt]) < 3) {
                if (temp == 0) {serial.send_AP(0,0,0);}
                else {serial.send_AP(0,buff_z[buffpnt],temp);}
           		}
            		else{
                  if (temp == 0) {serial.send_AP(0,0,0);}
                	else {serial.send_AP(buff_theta[buffpnt],buff_z[buffpnt],temp);}
            		}
            		cout << "Following! Going forward "<< temp << endl;
			}
			else if(state == FirstTime)
			{
		
				cout<<"No finding result!"<<endl;
			}
			else if(state == Nothing)
			{
				serial.send_AP(0,0,0);
				cout<<"Miss the target!"<<endl;
			}
		}
		else if(n>0)
		{
			cout<<"buffpnt = "<<buffpnt<<endl;
			if(state==Forward)
			{
				
            		cout << "Following! Going forward "<< buff_y[buffpnt] << endl;
			}
			else if(state == FirstTime)
			{
				
				cout<<"No finding result!"<<endl;
			}
			else if(state == Nothing)
			{
				
				cout<<"Miss the target!"<<endl;
			}
		}
          if(buffpnt == 15)
          {
            buffpnt =0;
          }
	}
}


