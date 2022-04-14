// Fill out your copyright notice in the Description page of Project Settings.


#include "Drone1Reference.h"

float drone1_xpos;
float drone1_ypos;
float drone1_zpos;
float drone1_xoffset;
float drone1_yoffset;
float drone1_zoffset;

float drone1_x;
float drone1_y;
float drone1_z;
// Sets default values
ADrone1Reference::ADrone1Reference()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADrone1Reference::BeginPlay()
{
	Super::BeginPlay();
	
}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback1 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DronePos1 = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (DronePos1.IsValid())
	{
		drone1_xpos = (DronePos1->pose.position.x);
		drone1_ypos = (DronePos1->pose.position.y);
		drone1_zpos = (DronePos1->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback1 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DroneOff1 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (DroneOff1.IsValid())
	{
		drone1_xoffset = (DroneOff1->x);
		drone1_yoffset = (DroneOff1->y);
		drone1_zoffset = (DroneOff1->z);
	}
	return;
};

// Called every frame
void ADrone1Reference::GetDrone1Location()
{
	DronePositionTopic1 = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic1 = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	//------------------------------------Drone 1 -----------------------------------------
	FString Drone1PosTopicName = "/uav1/mavros/local_position/pose";
	FString Drone1OffetTopicName = "/uav1/drone_offset";

	DronePositionTopic1->Init(rosinst->ROSIntegrationCore, Drone1PosTopicName, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic1->Init(rosinst->ROSIntegrationCore, Drone1OffetTopicName, TEXT("geometry_msgs/Vector3"));


	// Subscribe to the topic
	DronePositionTopic1->Subscribe(DroneCallback1);
	DroneOffsetTopic1->Subscribe(DroneOffsetCallback1);

	drone1_x = drone1_xpos + drone1_xoffset;
	drone1_y = drone1_ypos + drone1_yoffset;
	drone1_z = drone1_zpos + drone1_zoffset;

}

