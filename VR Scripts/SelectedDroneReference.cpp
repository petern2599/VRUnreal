// Fill out your copyright notice in the Description page of Project Settings.


#include "SelectedDroneReference.h"

extern FString DroneSelected;
float drone_xpos;
float drone_ypos;
float drone_zpos;
float drone_xoffset;
float drone_yoffset;
float drone_zoffset;

float drone_x;
float drone_y;
float drone_z;

// Sets default values
ASelectedDroneReference::ASelectedDroneReference()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASelectedDroneReference::BeginPlay()
{
	Super::BeginPlay();
	
}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (Concrete.IsValid())
	{
		drone_xpos = (Concrete->pose.position.x);
		drone_ypos = (Concrete->pose.position.y);
		drone_zpos = (Concrete->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (Concrete2.IsValid())
	{
		drone_xoffset = (Concrete2->x);
		drone_yoffset = (Concrete2->y);
		drone_zoffset = (Concrete2->z);
	}
	return;
};



// Called every frame
void ASelectedDroneReference::GetDroneLocation()
{
	//Super::Tick(DeltaTime);
	DronePositionTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	FString TopicName14 = DroneSelected + "/mavros/local_position/pose";
	FString TopicName15 = DroneSelected + "/drone_offset";

	DronePositionTopic->Init(rosinst->ROSIntegrationCore, TopicName14, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic->Init(rosinst->ROSIntegrationCore, TopicName15, TEXT("geometry_msgs/Vector3"));


	// Subscribe to the topic
	DronePositionTopic->Subscribe(DroneCallback);
	DroneOffsetTopic->Subscribe(DroneOffsetCallback);

	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Drone Location: %d"), drone_xpos));
	drone_x = drone_xpos + drone_xoffset;
	drone_y = drone_ypos + drone_yoffset;
	drone_z = drone_zpos + drone_zoffset;


}

