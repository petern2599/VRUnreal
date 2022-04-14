// Fill out your copyright notice in the Description page of Project Settings.


#include "Drone0Reference.h"

float drone0_xpos;
float drone0_ypos;
float drone0_zpos;
float drone0_xoffset;
float drone0_yoffset;
float drone0_zoffset;

float drone0_x;
float drone0_y;
float drone0_z;

// Sets default values
ADrone0Reference::ADrone0Reference()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADrone0Reference::BeginPlay()
{
	Super::BeginPlay();
	
}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback0 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DronePos0 = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (DronePos0.IsValid())
	{
		drone0_xpos = (DronePos0->pose.position.x);
		drone0_ypos = (DronePos0->pose.position.y);
		drone0_zpos = (DronePos0->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback0 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DroneOff0 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (DroneOff0.IsValid())
	{
		drone0_xoffset = (DroneOff0->x);
		drone0_yoffset = (DroneOff0->y);
		drone0_zoffset = (DroneOff0->z);
	}
	return;
};

// Called every frame
void ADrone0Reference::GetDrone0Location()
{

	DronePositionTopic0 = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic0 = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	//------------------------------------Drone 0 -----------------------------------------
	FString Drone0PosTopicName = "/uav0/mavros/local_position/pose";
	FString Drone0OffetTopicName = "/uav0/drone_offset";

	DronePositionTopic0->Init(rosinst->ROSIntegrationCore, Drone0PosTopicName, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic0->Init(rosinst->ROSIntegrationCore, Drone0OffetTopicName, TEXT("geometry_msgs/Vector3"));


	// Subscribe to the topic
	DronePositionTopic0->Subscribe(DroneCallback0);
	DroneOffsetTopic0->Subscribe(DroneOffsetCallback0);

	drone0_x = drone0_xpos + drone0_xoffset;
	drone0_y = drone0_ypos + drone0_yoffset;
	drone0_z = drone0_zpos + drone0_zoffset;

}
