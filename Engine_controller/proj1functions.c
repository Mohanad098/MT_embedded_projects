#include <stdio.h>
#define ENGINE_CONTROLLER 1

char system(){
	char System_choice;

	do{
		printf("a. Turn on the vehicle engine\n");
		printf("b. Turn off the vehicle engine\n");
		printf("c. Quit the system\n");
		scanf(" %c",&System_choice);
		if (System_choice=='b'){
			printf("Vehicle engine is turned off\n");
		}
	}while(System_choice!='a' && System_choice!='c');  // if user entered b or any other letter he will be asked the same questions again

	return System_choice;
}

int trafficLight(){
	char color;
	int speed;
	printf("Enter the traffic light data (G or O or R)\n");
	scanf(" %c",&color);

	switch(color){
	case 'G':
	case 'g': speed=100;
	break;
	case 'O':
	case 'o': speed=30;
	break;
	case 'R':
	case 'r': speed=0;
	break;
	default: printf("Invalid input");
	break;
	}
	return speed;
}

int room_temp(int *AC_state){
	int temp;
	printf("Enter room temperature\n");
	scanf("%d",&temp);
	if(temp<10||temp>30){
		*AC_state=1;    // AC turned on
		temp=20;
	}
	else{
		*AC_state=0;    // AC turned off
	}
	return temp;
}

#if ENGINE_CONTROLLER                            //bonus requirement
int engine_temp(int *engine_controller){
	int temp;
	printf("Enter engine temperature\n");
	scanf("%d",&temp);
	if (temp<100||temp>150){
		*engine_controller=1;    // Engine Temperature Controller turned ON
		temp=125;
	}
	else {
		*engine_controller=0;    // Engine Temperature Controller turned OFF
	}
	return temp;
}
#endif



