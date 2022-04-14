// Fill out your copyright notice in the Description page of Project Settings.


#include "RingWaypointActor.h"

// Sets default values
ARingWaypointActor::ARingWaypointActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARingWaypointActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
//void ARingWaypointActor::Tick(float DeltaTime)
//{
//	Super::Tick(DeltaTime);
//
//}

void ARingWaypointActor::SendDroneToWaypoint(FVector waypointLocation, int uasIndex)
{
	UE_LOG(LogTemp, Warning, TEXT("Sending Target Position"));
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Sending Target Position: %1.2f, %1.2f, %1.2f"), waypointLocation.X, waypointLocation.Y, waypointLocation.Z));

	// Initialize a topic
	VectorTopic = NewObject<UTopic>(UTopic::StaticClass());
	EnableTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	FString uasIndexStr = FString::FromInt(uasIndex);
	FString TopicName16 = "uav" + uasIndexStr + "/ring_target";
	FString TopicName17 = "uav0/ring_enable";
	

	VectorTopic->Init(rosinst->ROSIntegrationCore, TopicName16, TEXT("geometry_msgs/Vector3"));
	EnableTopic->Init(rosinst->ROSIntegrationCore, TopicName17, TEXT("std_msgs/String"));
	//Get Character Position

	//Publish to ROS
	TSharedPtr<ROSMessages::geometry_msgs::Vector3> Vector3Message(new ROSMessages::geometry_msgs::Vector3(waypointLocation.X / 100, waypointLocation.Y / 100, waypointLocation.Z / 100));

	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Ring Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Ring Disabled"));

	VectorTopic->Publish(Vector3Message);
	EnableTopic->Publish(EnableMessage);

}

void ARingWaypointActor::DisableDrone(int uasIndex)
{
	
	// Initialize a topic
	EnableTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	FString TopicName17 = "uav0/ring_enable";


	EnableTopic->Init(rosinst->ROSIntegrationCore, TopicName17, TEXT("std_msgs/String"));
	//Get Character Position

	//Publish to ROS

	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Ring Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Ring Disabled"));

	
	EnableTopic->Publish(DisableMessage);

}
