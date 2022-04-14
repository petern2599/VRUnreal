// Fill out your copyright notice in the Description page of Project Settings.


#include "Drone2Reference.h"

float drone2_xpos;
float drone2_ypos;
float drone2_zpos;
float drone2_xoffset;
float drone2_yoffset;
float drone2_zoffset;

float drone2_x;
float drone2_y;
float drone2_z;

// Sets default values
ADrone2Reference::ADrone2Reference()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADrone2Reference::BeginPlay()
{
	Super::BeginPlay();
	
}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback2 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DronePos2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (DronePos2.IsValid())
	{
		drone2_xpos = (DronePos2->pose.position.x);
		drone2_ypos = (DronePos2->pose.position.y);
		drone2_zpos = (DronePos2->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback2 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto DroneOff2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (DroneOff2.IsValid())
	{
		drone2_xoffset = (DroneOff2->x);
		drone2_yoffset = (DroneOff2->y);
		drone2_zoffset = (DroneOff2->z);
	}
	return;
};

// Called every frame
void ADrone2Reference::GetDrone2Location()
{
	DronePositionTopic2 = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic2 = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	//------------------------------------Drone 2 -----------------------------------------
	FString Drone2PosTopicName = "/uav2/mavros/local_position/pose";
	FString Drone2OffetTopicName = "/uav2/drone_offset";

	DronePositionTopic2->Init(rosinst->ROSIntegrationCore, Drone2PosTopicName, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic2->Init(rosinst->ROSIntegrationCore, Drone2OffetTopicName, TEXT("geometry_msgs/Vector3"));


	// Subscribe to the topic
	DronePositionTopic2->Subscribe(DroneCallback2);
	DroneOffsetTopic2->Subscribe(DroneOffsetCallback2);

	drone2_x = drone2_xpos + drone2_xoffset;
	drone2_y = drone2_ypos + drone2_yoffset;
	drone2_z = drone2_zpos + drone2_zoffset;
}

