#include <stdio.h>
#define ENGINE_CONTROLLER 1

#include "proj1.h"

int main(void){
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	char System_choice,sensors_menu;

	int vehicle_speed=0,roomTemp=20,AC_state=0,engineTemp=125,engine_controller_state=0;

	for(;;){

		System_choice=system();

		if (System_choice=='c'){
			printf("System will close. Goodbye!\n");
			return 0;
		}

		else if(System_choice=='a'){
			printf("Vehicle engine is turned on\n");
			while(System_choice=='a'){
				printf("a. Turn off the engine\n");
				printf("b. Set the traffic light color\n");
				printf("c. Set the room temperature (Temperature Sensor)\n");
#if ENGINE_CONTROLLER
				printf("d. Set the engine temperature (Engine Temperature Sensor)\n");
#endif
				scanf(" %c",&sensors_menu);

				if (sensors_menu!='a'){

					if (sensors_menu=='b'){
						vehicle_speed=trafficLight();
					}
					else if (sensors_menu=='c'){
						roomTemp=room_temp(&AC_state);
					}
#if ENGINE_CONTROLLER
					else if (sensors_menu=='d'){
						engineTemp=engine_temp(&engine_controller_state);
					}
#endif

					if(vehicle_speed==30){
						AC_state=1;
						roomTemp*=(5/4)+1;
#if ENGINE_CONTROLLER
						engine_controller_state=1;
						engineTemp*=(5/4)+1;
#endif
					}
					// Displaying states as long as engine is on:

					printf("Engine is turned on\n");                                 // Engine
					if (AC_state==0){
						printf("AC turned off\n");
					}
					else if (AC_state==1){
						printf("AC turned on\n");
					}                                                                // AC
					printf("Vehicle speed = %d\n",vehicle_speed);                    // Vehicle speed
					printf("Room temperature = %d\n",roomTemp);                      // room temp
#if ENGINE_CONTROLLER
					if (engine_controller_state==0){
						printf("Engine temperature controller is turned off\n");
					}
					else if (engine_controller_state==1){
						printf("Engine temperature controller is turned On\n");
					}                                                                // engine temp controller
					printf("Engine temperature = %d\n",engineTemp);                  // engine temp
#endif
				}
				else {                      // if user chose to turn off engine
					break;
				}
			}
		}


	}
	return 0;
}
