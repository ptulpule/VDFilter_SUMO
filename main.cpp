#include <iostream>
#include <valarray>
#include "MainVD.h"
#include "Client.h"
#include <fstream>
int main() {
	MainVD vd;
	Client client;
	int count = 0;
	double ue_X = 0, ue_Y = 0, ue_V = 0;
	double psi = 0;
	int isVeh0 = 0;
	int SumoCounter = 0;  //Number of sumo steps for car0
	libsumo::TraCIPosition sumo_pos;
	double sumo_pos_pre_y = 0;
	double sumo_pos_pre_x= 0;
	double sumo_pos_mid_x = 0;
	double sumo_pos_mid_y = 0;
	double sumo_spd_pre = 0;
	double sumo_spd_mid = 0;
	double sumo_angle;
    double dist;
	std::ofstream myfile;
	myfile.open("example.csv");

	try {
		client.connect("localhost", 24000);

		while (client.simulation.getMinExpectedNumber() > 0) {
		    count++;

		    if (isVeh0==1 && SumoCounter>1){    //pt addition
		        //vd.VD_filter(sumo_pos.x, sumo_pos.y, sumo_angle,&psi, &ue_X, &ue_Y, &ue_V);
		        psi = atan2(sumo_pos.y-ue_Y,sumo_pos.x-ue_X);
		        dist = sqrt(pow((ue_X-sumo_pos.x),2)+pow((ue_Y-sumo_pos.y),2));
                ue_X = ue_X+0.1*dist*cos(psi);
                ue_Y = ue_Y+0.1*dist*sin(psi);
		    }


		    if(count % 10 == 0){
		        client.simulationStep();
		        if (client.vehicle.getIDCount()==1){
		            SumoCounter++;
		        }

				if (client.vehicle.getIDCount()==1 && isVeh0==0){    // pt addition: detect if vehicle with id car0 is generated in SUMO
				    sumo_pos = client.vehicle.getPosition("car0");
				    sumo_pos_pre_x = sumo_pos.x;
					sumo_pos_pre_y = sumo_pos.y;
					ue_X = sumo_pos.x;
					ue_Y = sumo_pos.y;
					sumo_spd_pre = client.vehicle.getSpeed("car0");
					isVeh0=1;

				}
                if(SumoCounter==1){
                    sumo_pos = client.vehicle.getPosition("car0");
                    sumo_pos_mid_x = sumo_pos.x;
                    sumo_pos_mid_y= sumo_pos.y;
                    sumo_spd_mid = client.vehicle.getSpeed("car0");
                }

                if(SumoCounter ==2){
                    sumo_pos = client.vehicle.getPosition("car0");
                    sumo_angle = atan2(sumo_pos.y-sumo_pos_pre_y,sumo_pos.x-sumo_pos_pre_x);
                }
				if(SumoCounter>2){
				    sumo_pos_pre_x = sumo_pos_mid_x;
				    sumo_pos_pre_y = sumo_pos_mid_y;
				    //ue_X = sumo_pos_pre_x;
				    //ue_Y = sumo_pos_pre_y;
				    ue_V = sumo_spd_pre;
				    sumo_spd_pre = sumo_spd_mid;
				    sumo_spd_mid = client.vehicle.getSpeed("car0");
				    sumo_pos_mid_x = sumo_pos.x;
				    sumo_pos_mid_y = sumo_pos.y;
				    sumo_pos = client.vehicle.getPosition("car0");
				    sumo_angle = atan2(sumo_pos.y-sumo_pos_pre_y,sumo_pos.x-sumo_pos_pre_x);
				}



			}
		    //all_ue_x[count] = ue_X;
		    //all_ue_y[count] = ue_Y;
		    std::cout<< ue_X<<","<<ue_Y<< ","<<ue_V<< ","<<sumo_pos.x<< ","<< sumo_pos.y<<std::endl;

		}

	}catch (tcpip::SocketException& e) {
		std::cout << "#Error while connecting: " << e.what();
	}


	myfile.close();
	std::cout << "Hello, World!" << std::endl;
	return 0;
}
