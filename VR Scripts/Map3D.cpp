// Fill out your copyright notice in the Description page of Project Settings.


#include "Map3D.h"

extern float drone_x;
extern float drone_y;
extern float drone_z;

extern float drone0_x;
extern float drone0_y;
extern float drone0_z;

extern float drone1_x;
extern float drone1_y;
extern float drone1_z;

extern float drone2_x;
extern float drone2_y;
extern float drone2_z;

// Sets default values
AMap3D::AMap3D()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AMap3D::BeginPlay()
{
	Super::BeginPlay();
	
}

void AMap3D::GetAllDroneLocations()
{

	Drone0_LocationX = drone0_x;
	Drone0_LocationY = drone0_y;
	Drone0_LocationZ = drone0_z;

	Drone1_LocationX = drone1_x;
	Drone1_LocationY = drone1_y;
	Drone1_LocationZ = drone1_z;

	Drone2_LocationX = drone2_x;
	Drone2_LocationY = drone2_y;
	Drone2_LocationZ = drone2_z;

}

