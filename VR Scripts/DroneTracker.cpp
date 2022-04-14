// Fill out your copyright notice in the Description page of Project Settings.


#include "DroneTracker.h"
#include "DrawDebugHelpers.h"
#include <cmath>

extern float drone_xpos;
extern float drone_ypos;
extern float drone_zpos;
extern float drone_xoffset;
extern float drone_yoffset;
extern float drone_zoffset;
extern FString DroneSelected;

// Sets default values
ADroneTracker::ADroneTracker()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADroneTracker::BeginPlay()
{
	Super::BeginPlay();
	
}

void ADroneTracker::TrackDrone(FVector actorLocation)
{
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Before")));
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Drone Name: %s"), *DroneSelected));

	FVector droneLocation = FVector((drone_ypos + drone_yoffset) * 100, (drone_xpos + drone_xoffset) * 100, (drone_zpos + drone_zoffset) * 100);
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Drone Location: %s"), *droneLocation.ToString()));
	FVector droneFromActor = droneLocation - actorLocation;
	float droneLocationMagnitude = sqrt(pow(droneFromActor.X, 2) + pow(droneFromActor.Y, 2) + pow(droneFromActor.Z, 2));
	FVector droneLocationUnitVector = FVector((droneFromActor.X / droneLocationMagnitude) * 5, (droneFromActor.Y / droneLocationMagnitude) * 5, (droneFromActor.Z / droneLocationMagnitude)) * 5;

	DrawDebugBox(GetWorld(), FVector(droneLocation.X, droneLocation.Y, droneLocation.Z - 50), FVector(100, 100, 100), FColor(0, 255, 127), false, 1.0f, 0, 7.0f);
	DrawDebugDirectionalArrow(GetWorld(), actorLocation, (actorLocation + droneLocationUnitVector), 10.f, FColor(0, 255, 127), false, 1.0f, 7.0f);
}

//std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback6 = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
//	if (Concrete0.IsValid())
//	{
//		drone_select6 = (Concrete0->_Data);
//	}
//	return;
//};

//std::function<void(TSharedPtr<FROSBaseMsg>)> DroneLocationCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
//	if (Concrete.IsValid())
//	{
//		drone_xpos3 = (Concrete->pose.position.x);
//		drone_ypos3 = (Concrete->pose.position.y);
//		drone_zpos3 = (Concrete->pose.position.z);
//	}
//	return;
//};
//
//std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback3 = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
//	if (Concrete2.IsValid())
//	{
//		drone_xoffset3 = (Concrete2->x);
//		drone_yoffset3 = (Concrete2->y);
//		drone_zoffset3 = (Concrete2->z);
//	}
//	return;
//};

// Called every frame
//void ADroneTracker::Tick(float DeltaTime)
//{
//	Super::Tick(DeltaTime);
//	DroneSelectionTopic6 = NewObject<UTopic>(UTopic::StaticClass());
//	DroneLocationTopic = NewObject<UTopic>(UTopic::StaticClass());
//	DroneOffsetTopic = NewObject<UTopic>(UTopic::StaticClass());
//	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//
//	DroneSelectionTopic6->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
//	DroneSelectionTopic6->Subscribe(DroneSelectionCallback6);
//
//	TopicName16 = drone_select6 + "/mavros/local_position/pose";
//	TopicName17 = drone_select6 + "/drone_offset";
//
//	DroneLocationTopic->Init(rosinst->ROSIntegrationCore, TopicName16, TEXT("geometry_msgs/PoseStamped"));
//	DroneOffsetTopic->Init(rosinst->ROSIntegrationCore, TopicName17, TEXT("geometry_msgs/Vector3"));
//
//
//	// Subscribe to the topic
//	DroneLocationTopic->Subscribe(DroneLocationCallback);
//	DroneOffsetTopic->Subscribe(DroneOffsetCallback3);
//}

