// Fill out your copyright notice in the Description page of Project Settings.


#include "DroneLocationReferenceActor.h"

float droneref_xpos;
float droneref_ypos;
float droneref_zpos;
float droneref_xoffset;
float droneref_yoffset;
float droneref_zoffset;

float droneref_x;
float droneref_y;
float droneref_z;

// Sets default values
ADroneLocationReferenceActor::ADroneLocationReferenceActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADroneLocationReferenceActor::BeginPlay()
{
	Super::BeginPlay();

}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneRefCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DronePos0 = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (DronePos0.IsValid())
	{
		droneref_xpos = (DronePos0->pose.position.x);
		droneref_ypos = (DronePos0->pose.position.y);
		droneref_zpos = (DronePos0->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneRefOffsetCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DroneOff0 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (DroneOff0.IsValid())
	{
		droneref_xoffset = (DroneOff0->x);
		droneref_yoffset = (DroneOff0->y);
		droneref_zoffset = (DroneOff0->z);
	}
	return;
};

// Called every frame
void ADroneLocationReferenceActor::GetDroneLocation()
{

	DronePositionTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	//------------------------------------Drone 0 -----------------------------------------
	FString Drone0PosTopicName = "/uav0/mavros/local_position/pose";
	FString Drone0OffetTopicName = "/uav0/drone_offset";

	DronePositionTopic->Init(rosinst->ROSIntegrationCore, Drone0PosTopicName, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic->Init(rosinst->ROSIntegrationCore, Drone0OffetTopicName, TEXT("geometry_msgs/Vector3"));


	// Subscribe to the topic
	DronePositionTopic->Subscribe(DroneRefCallback);
	DroneOffsetTopic->Subscribe(DroneRefOffsetCallback);

	droneref_x = droneref_xpos + droneref_xoffset;
	droneref_y = droneref_ypos + droneref_yoffset;
	droneref_z = droneref_zpos + droneref_zoffset;

}
